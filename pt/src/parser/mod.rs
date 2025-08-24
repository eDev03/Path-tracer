use std::{collections::HashMap, io::Read, str::FromStr};

use itertools::Itertools;
use math::Matrix4f32;

use crate::{
    geom::triangle_cross,
    io_bridge::{self},
    lighting,
    spaces::{Point2, Point3Object, Point3World, RGBf32, Vec3Object, Vec3World},
};

type Mat4 = Matrix4f32<(), ()>;
type Transform = Matrix4f32<(), ()>;
type Defaults<'key, 'input> = HashMap<&'key str, &'input str>;

pub struct ParseResult {
    pub output_name: String,
    pub integrator: io_bridge::Integrator,
    pub camera: io_bridge::CameraOptions,
    pub meshes: Box<[io_bridge::Mesh]>,
    pub materials: Box<[io_bridge::Material]>,
    pub envmap: Option<lighting::Envmap>,
}

pub fn parse(scene_collection: &str, scene_base_path: &str) -> ParseResult {
    let root = std::path::Path::new(scene_collection).join(scene_base_path);

    let mut file = find_file(&root);
    let mut content = String::new();
    assert_eq!(file.read_to_string(&mut content).unwrap(), content.len());
    let xml = roxmltree::Document::parse(&content).unwrap();
    let xml_scene = xml.descendants().nth(1).unwrap();
    assert_eq!(xml_scene.tag_name().name(), "scene");
    assert_eq!(xml_scene.attribute("version").unwrap(), "3.0.0");
    let output_name = xml_scene
        .attribute("output")
        .unwrap_or("out.png")
        .to_owned();

    let mut defaults = Defaults::new();
    let mut bsdfs = HashMap::new();
    let mut shapes = HashMap::new();
    let mut integrator = Integrator::default();
    let mut sensor = Sensor::default();
    let mut envmap = None;

    for node in children(xml_scene) {
        match parse_node(node, &defaults) {
            Node::Default(k, v) => {
                defaults.insert(k, v);
            }
            Node::Integrator(node_integrator) => {
                integrator = node_integrator;
            }
            Node::Sensor(node_sensor) => {
                sensor = node_sensor;
            }
            Node::BSDF(id, bsdf) => {
                bsdfs.insert(id, bsdf);
            }
            Node::Shape(id, shape) => {
                shapes.insert(id, shape);
            }
            Node::SunksyEmitter(SunskyEmitter {
                turbidity,
                latitude,
                longitude,
                timezone,
                year,
                month,
                day,
                hour,
                minute,
                second,
                sun_direction,
                sun_scale,
                sky_scale,
                sun_aperture,
                to_world,
            }) => {
                envmap = Some(lighting::Envmap::from_sunsky(
                    turbidity,
                    latitude,
                    longitude,
                    timezone,
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                    sun_direction,
                    sun_scale,
                    sky_scale,
                    sun_aperture,
                    to_world.change_spaces(),
                ))
            }
            _ => unused(node),
        }
    }

    let mut index_map = HashMap::new();
    let mut materials = Vec::new();

    for (id, bsdf) in bsdfs {
        let index = materials.len();
        index_map.insert(id, index);
        materials.push(match bsdf {
            BSDF::Diffuse(Diffuse { reflectance }) => io_bridge::Material::Lambertian {
                albedo: io_bridge::Spectrum::RGB(reflectance),
            },
            BSDF::Conductor(Conductor { roughness, eta, k }) => io_bridge::Material::Conductor {
                roughness,
                eta: io_bridge::Spectrum::RGB(eta),
                k: io_bridge::Spectrum::RGB(k),
            },
            BSDF::Dielectric(Dielectric { ext_ior, int_ior }) => io_bridge::Material::Dielectric {
                eta: io_bridge::Spectrum::Constant(int_ior / ext_ior),
            },
        });
    }

    let integrator = match integrator {
        Integrator::Path(PathIntegrator { max_depth, mode }) => {
            io_bridge::Integrator::PathIntegrator(io_bridge::PathIntegrator {
                max_depth,
                mode: mode.io_bridge(),
            })
        }
        Integrator::LightPath(LightPathIntegrator { max_depth }) => {
            io_bridge::Integrator::LightPathIntegrator(io_bridge::LightPathIntegrator { max_depth })
        }
        Integrator::PathGuiding(PathGuidingIntegrator {
            max_depth,
            spatial_threshold,
            directional_threshold,
            mode,
        }) => io_bridge::Integrator::PathGuidingIntegrator(io_bridge::PathGuidingIntegrator {
            max_depth,
            spatial_threshold,
            directional_threshold,
            spatial_filter: io_bridge::Filter::Stochastic,
            directional_filter: io_bridge::Filter::Box,
            mode: mode.io_bridge(),
        }),
    };

    let camera = io_bridge::CameraOptions {
        fov: sensor.fov,
        image_size: [sensor.film.width as usize, sensor.film.height as usize],
        world_to_camera: sensor.to_world.inverse().change_spaces(),
        spp: sensor.sampler.sample_count as usize,
    };

    let mut serialized_data = HashMap::new();

    let meshes: Box<[io_bridge::Mesh]> = shapes
        .into_iter()
        .map(|(_id, shape)| {
            let emission = shape.emitter.map(|e| e.rgb).unwrap_or(RGBf32::ZERO);
            let material_index = shape
                .properties
                .iter()
                .find_map(|prop| index_map.get(prop.as_str()))
                .copied();
            let (points, normals, mut triangles) = match shape.description {
                MeshDescription::TriangulatedMesh(TriangulatedMesh {
                    triangles,
                    points,
                    normals,
                }) => (points, normals, triangles),
                MeshDescription::ObjMesh(ObjMesh {
                    filename,
                    face_normals,
                }) => {
                    let file_path = root.join(filename);
                    let (p, mut n, t) = parse_obj(file_path);
                    if face_normals {
                        n = Default::default()
                    }
                    (p, n, t)
                }
                MeshDescription::PlyMesh(PlyMesh {
                    filename,
                    face_normals,
                }) => {
                    let path = root.join(filename);
                    let mut file = std::fs::File::open(path).unwrap();
                    let mut data = Vec::new();
                    assert_eq!(file.read_to_end(&mut data).unwrap(), data.len());
                    let (p, mut n, t) = parse_ply(&data);
                    if face_normals {
                        n = Default::default()
                    }
                    (p, n, t)
                }
                MeshDescription::Serialized(Serialized {
                    filename,
                    shape_index,
                }) => {
                    let data = serialized_data.entry(filename).or_insert_with(|| {
                        let mut file = std::fs::File::open(root.join(filename)).unwrap();
                        let mut data = Vec::new();
                        assert_eq!(file.read_to_end(&mut data).unwrap(), data.len());
                        let format = u16::from_le_bytes([data[0], data[1]]);
                        let version = u16::from_le_bytes([data[2], data[3]]);
                        assert_eq!(format, 0x041c);
                        assert!(version == 3 || version == 4);
                        data
                    });
                    parse_serialized(data, shape_index as usize)
                }
            };
            let transform = shape.transform.change_spaces();
            let points = points.into_iter().map(|p| transform * p).collect();
            let m_inv = shape.transform.inverse();
            let normals = normals
                .into_iter()
                .map(|n| {
                    [
                        m_inv.cols[0][0] * n.x + m_inv.cols[1][0] * n.y + m_inv.cols[2][0] * n.z,
                        m_inv.cols[0][1] * n.x + m_inv.cols[1][1] * n.y + m_inv.cols[2][1] * n.z,
                        m_inv.cols[0][2] * n.x + m_inv.cols[1][2] * n.y + m_inv.cols[2][2] * n.z,
                    ]
                    .into()
                })
                .collect();

            if transform_swaps_handedness(shape.transform) {
                for tri in triangles.iter_mut() {
                    tri.reverse();
                }
            }

            io_bridge::Mesh {
                points,
                normals,
                triangles,
                emission,
                material_index,
            }
        })
        .collect();

    ParseResult {
        output_name,
        integrator,
        camera,
        meshes,
        materials: materials.into_boxed_slice(),
        envmap,
    }
}

fn parse_node<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Node<'input> {
    match node.tag_name().name() {
        "default" => Node::Default(
            attribute(node, "name", defaults),
            attribute(node, "value", defaults),
        ),
        "integrator" => Node::Integrator(parse_integrator(node, defaults)),
        "sensor" => Node::Sensor(parse_sensor(node, defaults)),
        "transform" => Node::Transform(
            attribute(node, "name", defaults),
            parse_transform(node, defaults),
        ),
        "scale" => Node::Scale(parse_scale(node, defaults)),
        "translate" => Node::Translate(parse_translate(node, defaults)),
        "matrix" => Node::Matrix(parse_matrix(attribute(node, "value", defaults))),
        "string" => Node::String(
            attribute(node, "name", defaults),
            attribute(node, "value", defaults),
        ),
        "boolean" => Node::Bool(
            attribute(node, "name", defaults),
            attribute(node, "value", defaults).parse().unwrap(),
        ),
        "integer" => Node::Integer(
            attribute(node, "name", defaults),
            attribute(node, "value", defaults).parse().unwrap(),
        ),
        "float" => Node::Float(
            attribute(node, "name", defaults),
            attribute(node, "value", defaults).parse().unwrap(),
        ),
        "point" => Node::Point3(
            attribute(node, "name", defaults),
            parse_point(node, defaults),
        ),
        "vector" => Node::Vector3(
            attribute(node, "name", defaults),
            parse_vector(node, defaults, 0.0),
        ),
        "lookat" => Node::Lookat {
            target: parse_array(attribute(node, "target", defaults)),
            origin: parse_array(attribute(node, "origin", defaults)),
            up: parse_array(attribute(node, "up", defaults)),
        },
        "rgb" => Node::RGB(attribute(node, "name", defaults), parse_rgb(node, defaults)),
        "sampler" => Node::Sampler(parse_sampler(node, defaults)),
        "film" => Node::Film(parse_film(node, defaults)),
        "rfilter" => Node::String("rfilter", "unused"),
        "bsdf" => Node::BSDF(attribute(node, "id", defaults), parse_bsdf(node, defaults)),
        "shape" => Node::Shape(attribute(node, "id", defaults), parse_shape(node, defaults)),
        "ref" => Node::Ref(attribute(node, "id", defaults)),
        "emitter" => {
            let emitter_type = attribute(node, "type", defaults);
            match emitter_type {
                "area" => Node::AreaEmitter(parse_area_emitter(node, defaults)),
                "sunsky" => Node::SunksyEmitter(parse_sunsky_emitter(node, defaults)),
                _ => panic!("unknown emitter type: {emitter_type}"),
            }
        }
        "rotate" => {
            let (axis, angle) = parse_rotate(node, defaults);
            Node::Rotate(axis, angle)
        }
        _ => panic!("unknown tag name: {}", node.tag_name().name()),
    }
}

fn parse_integrator<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Integrator {
    let integrator_type = attribute(node, "type", defaults);
    fn parse_mode(mode: &str) -> PathIntegratorMode {
        match mode {
            "PT" => PathIntegratorMode::PT,
            "NEE" => PathIntegratorMode::NEE,
            "MIS" => PathIntegratorMode::MIS,
            _ => panic!("unknown integrator mode: {}", mode),
        }
    }
    match integrator_type {
        "path" => {
            let mut integrator = PathIntegrator::default();
            for node in children(node) {
                match parse_node(node, defaults) {
                    Node::Integer("max_depth", val) => integrator.max_depth = val as usize,
                    Node::String("mode", val) => integrator.mode = parse_mode(val),
                    _ => unused(node),
                }
            }
            Integrator::Path(integrator)
        }
        "lightpath" => {
            let mut integrator = LightPathIntegrator::default();
            for node in children(node) {
                match parse_node(node, defaults) {
                    Node::Integer("max_depth", val) => integrator.max_depth = val as usize,
                    _ => unused(node),
                }
            }
            Integrator::LightPath(integrator)
        }
        "guided" => {
            let mut integrator = PathGuidingIntegrator::default();
            for node in children(node) {
                match parse_node(node, defaults) {
                    Node::Float("spatial_threashold", val) => integrator.spatial_threshold = val,
                    Node::Integer("max_depth", val) => integrator.max_depth = val as usize,
                    Node::String("mode", val) => integrator.mode = parse_mode(val),
                    _ => unused(node),
                }
            }
            Integrator::PathGuiding(integrator)
        }
        _ => panic!("unkown integrator type: {}", integrator_type),
    }
}

fn parse_sensor<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Sensor {
    let sensor_type = attribute(node, "type", defaults);
    assert_eq!(sensor_type, "perspective");
    let mut sensor = Sensor::default();
    for node in children(node) {
        match parse_node(node, defaults) {
            Node::Float("fov", val) => sensor.fov = val,
            Node::Transform("to_world", val) => sensor.to_world = val,
            Node::Sampler(val) => sensor.sampler = val,
            Node::Film(val) => sensor.film = val,
            _ => unused(node),
        }
    }
    sensor
}

fn parse_film<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Film {
    let mut film = Film::default();
    for node in children(node) {
        match parse_node(node, defaults) {
            Node::Integer("width", val) => film.width = val,
            Node::Integer("height", val) => film.height = val,
            _ => unused(node),
        }
    }
    film
}

fn parse_sampler<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Sampler {
    let mut sampler = Sampler::default();
    for node in children(node) {
        match parse_node(node, defaults) {
            Node::Integer("sample_count", val) => sampler.sample_count = val,
            _ => unused(node),
        }
    }
    sampler
}

fn parse_bsdf<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> BSDF {
    let bsdf_type = attribute(node, "type", defaults);
    match bsdf_type {
        "twosided" => {
            let child = first_child(node);
            assert_eq!(child.tag_name().name(), "bsdf");
            parse_bsdf(child, defaults)
        }
        "diffuse" => {
            let mut diffuse = Diffuse::default();
            for node in children(node) {
                match parse_node(node, defaults) {
                    Node::RGB("reflectance", val) => diffuse.reflectance = val,
                    _ => unused(node),
                }
            }
            BSDF::Diffuse(diffuse)
        }
        "roughconductor" => {
            let mut conductor = Conductor::default();
            for node in children(node) {
                match parse_node(node, defaults) {
                    Node::Float("alpha", val) => conductor.roughness = val,
                    Node::RGB("eta", val) => conductor.eta = val,
                    Node::RGB("k", val) => conductor.k = val,
                    _ => unused(node),
                }
            }
            BSDF::Conductor(conductor)
        }
        "dielectric" => {
            let mut dielectric = Dielectric::default();
            for node in children(node) {
                match parse_node(node, defaults) {
                    Node::Float("int_ior", val) => dielectric.int_ior = val,
                    Node::Float("ext_ior", val) => dielectric.ext_ior = val,
                    _ => unused(node),
                }
            }
            BSDF::Dielectric(dielectric)
        }
        _ => panic!("unknown bsdf type: {}", bsdf_type),
    }
}

fn parse_rgb<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> RGBf32 {
    let rgb = parse_array(attribute(node, "value", defaults));
    rgb.into()
}

fn parse_matrix(s: &str) -> Mat4 {
    Mat4::from_rows_components(
        s.split(" ")
            .map(|i| i.parse().unwrap())
            .collect_array()
            .unwrap(),
    )
}

#[track_caller]
fn parse_array<const N: usize, T: FromStr>(s: &str) -> [T; N]
where
    <T as FromStr>::Err: std::fmt::Debug,
{
    s.split(", ")
        .map(|i| i.parse().unwrap())
        .collect_array()
        .unwrap()
}

fn parse_point<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> [f32; 3] {
    let x = attribute(node, "x", defaults).parse().unwrap();
    let y = attribute(node, "y", defaults).parse().unwrap();
    let z = attribute(node, "z", defaults).parse().unwrap();
    [x, y, z]
}

fn parse_vector<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
    default: f32,
) -> [f32; 3] {
    if let Some(val) = attribute_checked(node, "value", defaults) {
        return parse_array(val);
    };
    let x = attribute_checked(node, "x", defaults)
        .map(|i| i.parse().unwrap())
        .unwrap_or(default);
    let y = attribute_checked(node, "y", defaults)
        .map(|i| i.parse().unwrap())
        .unwrap_or(default);
    let z = attribute_checked(node, "z", defaults)
        .map(|i| i.parse().unwrap())
        .unwrap_or(default);
    [x, y, z]
}

fn parse_shape<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Shape<'input> {
    let shape_type = attribute(node, "type", defaults);
    let mut emitter = None;
    let mut properties = Vec::default();
    let mut transform = Mat4::identity();
    let mut triangulated_mesh = TriangulatedMesh::default();
    let mut obj_mesh = ObjMesh::default();
    let mut ply_mesh = PlyMesh::default();
    let mut serialized = Serialized::default();
    match shape_type {
        "rectangle" => {
            triangulated_mesh.points = Box::new([
                Point3Object::new(-1.0, -1.0, 0.0),
                Point3Object::new(1.0, -1.0, 0.0),
                Point3Object::new(1.0, 1.0, 0.0),
                Point3Object::new(-1.0, 1.0, 0.0),
            ]);
            triangulated_mesh.triangles = Box::new([[2, 1, 0], [0, 3, 2]]);
        }
        "cube" => {
            triangulated_mesh.points = Box::new([
                Point3Object::new(-1.0, -1.0, -1.0),
                Point3Object::new(1.0, -1.0, -1.0),
                Point3Object::new(1.0, -1.0, 1.0),
                Point3Object::new(-1.0, -1.0, 1.0),
                Point3Object::new(-1.0, 1.0, -1.0),
                Point3Object::new(1.0, 1.0, -1.0),
                Point3Object::new(1.0, 1.0, 1.0),
                Point3Object::new(-1.0, 1.0, 1.0),
            ]);
            triangulated_mesh.triangles = Box::new([
                [2, 1, 0],
                [0, 3, 2],
                [0, 1, 5],
                [5, 4, 0],
                [1, 2, 6],
                [6, 5, 1],
                [2, 3, 7],
                [7, 6, 2],
                [3, 0, 4],
                [4, 7, 3],
                [4, 5, 6],
                [6, 7, 4],
            ]);
        }
        "sphere" => {
            const ICO_SPHERE_PLY: &[u8] = include_bytes!("icosphere_6.ply");
            (
                triangulated_mesh.points,
                triangulated_mesh.normals,
                triangulated_mesh.triangles,
            ) = parse_ply(ICO_SPHERE_PLY);
        }
        "obj" | "ply" | "serialized" => {}
        _ => panic!("unknown shape type: {}", shape_type),
    }
    for node in children(node) {
        match (shape_type, parse_node(node, defaults)) {
            (_, Node::Transform("to_world", val)) => transform = val.change_spaces(),
            (_, Node::AreaEmitter(val)) => emitter = Some(val),
            (_, Node::Ref(id)) => properties.push(id.to_owned()),
            ("obj", Node::String("filename", val)) => obj_mesh.filename = val,
            ("obj", Node::Bool("face_normals", val)) => obj_mesh.face_normals = val,
            ("ply", Node::String("filename", val)) => ply_mesh.filename = val,
            ("sphere", Node::Float("radius", val)) => {
                for p in triangulated_mesh.points.iter_mut() {
                    *p *= val;
                }
            }
            ("sphere", Node::Point3("center", val)) => {
                for p in triangulated_mesh.points.iter_mut() {
                    *p += val.into();
                }
            }
            ("serialized", Node::String("filename", val)) => serialized.filename = val,
            ("serialized", Node::Integer("shape_index", val)) => serialized.shape_index = val,
            _ => unused(node),
        }
    }

    match shape_type {
        "rectangle" | "cube" | "sphere" => Shape {
            transform,
            emitter,
            properties,
            description: MeshDescription::TriangulatedMesh(triangulated_mesh),
        },
        "obj" => Shape {
            transform,
            emitter,
            properties,
            description: MeshDescription::ObjMesh(obj_mesh),
        },
        "ply" => Shape {
            transform,
            emitter,
            properties,
            description: MeshDescription::PlyMesh(ply_mesh),
        },
        "serialized" => Shape {
            transform,
            emitter,
            properties,
            description: MeshDescription::Serialized(serialized),
        },
        _ => panic!("i forgor {}", shape_type),
    }
}

fn transform_swaps_handedness(transform: Transform) -> bool {
    // upper 3x3 determinant
    let det = transform.cols[0].x.mul_add(
        transform.cols[1].y.mul_add(
            transform.cols[2].z,
            -(transform.cols[2].y * transform.cols[1].z),
        ),
        -(transform.cols[1].x.mul_add(
            transform.cols[0].y.mul_add(
                transform.cols[2].z,
                -(transform.cols[2].y * transform.cols[0].z),
            ),
            -(transform.cols[2].x
                * transform.cols[0].y.mul_add(
                    transform.cols[1].z,
                    -(transform.cols[1].y * transform.cols[0].z),
                )),
        )),
    );
    det < 0.0
}

fn parse_transform<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> Transform {
    let mut transform = Transform::identity();
    for node in children(node) {
        match parse_node(node, defaults) {
            Node::Matrix(val) => transform = val,
            Node::Scale(val) => transform = Mat4::from_scale(val.into()) * transform,
            Node::Translate(val) => transform = Mat4::from_translation(val.into()) * transform,
            Node::Rotate(axis, angle) => {
                transform = Mat4::from_rotation(axis.into(), angle.to_radians()) * transform
            }
            Node::Lookat { target, origin, up } => {
                let target = Point3World::from(target);
                let origin = Point3World::from(origin);
                let up = Vec3World::from(up);
                let fwd = (target - origin).normalized();
                let left = (up.cross(fwd)).normalized();
                let alt_up = (fwd.cross(left)).normalized();
                transform = Mat4::from_rows_components([
                    left.x, alt_up.x, fwd.x, origin.x, left.y, alt_up.y, fwd.y, origin.y, left.z,
                    alt_up.z, fwd.z, origin.z, 0.0, 0.0, 0.0, 1.0,
                ]) * transform;
            }
            _ => unused(node),
        }
    }
    transform
}

fn parse_rotate<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> ([f32; 3], f32) {
    let x = attribute_checked(node, "x", defaults)
        .map(|s| s.parse().unwrap())
        .unwrap_or(0.0);
    let y = attribute_checked(node, "y", defaults)
        .map(|s| s.parse().unwrap())
        .unwrap_or(0.0);
    let z = attribute_checked(node, "z", defaults)
        .map(|s| s.parse().unwrap())
        .unwrap_or(0.0);
    let angle = attribute(node, "angle", defaults).parse().unwrap();
    ([x, y, z], angle)
}

fn parse_translate<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> [f32; 3] {
    parse_vector(node, defaults, 0.0)
}

fn parse_scale<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> [f32; 3] {
    if let Some(f) = attribute_checked(node, "value", defaults).and_then(|s| s.parse().ok()) {
        return [f; 3];
    }
    parse_vector(node, defaults, 1.0)
}

fn parse_area_emitter<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> AreaEmitter {
    let mut emitter = AreaEmitter::default();
    for node in children(node) {
        match parse_node(node, defaults) {
            Node::RGB("radiance", val) => emitter.rgb = val,
            _ => unused(node),
        }
    }
    emitter
}

fn parse_sunsky_emitter<'input>(
    node: roxmltree::Node<'input, 'input>,
    defaults: &Defaults<'_, 'input>,
) -> SunskyEmitter {
    let mut emitter = SunskyEmitter::default();
    for node in children(node) {
        match parse_node(node, defaults) {
            Node::Float("hour", val) => emitter.hour = val,
            Node::Float("turbidity", val) => emitter.turbidity = val,
            Node::Float("latitude", val) => emitter.latitude = val,
            Node::Float("longitude", val) => emitter.longitude = val,
            Node::Transform("to_world", val) => emitter.to_world = val,
            Node::Vector3("sun_direction", val) => emitter.sun_direction = Some(val.into()),

            _ => unused(node),
        }
    }
    emitter
}

#[derive(Debug)]
enum Node<'input> {
    Default(&'input str, &'input str),
    Integrator(Integrator),
    Sensor(Sensor),
    String(&'input str, &'input str),
    Bool(&'input str, bool),
    Integer(&'input str, i32),
    Float(&'input str, f32),
    Point3(&'input str, [f32; 3]),
    Vector3(&'input str, [f32; 3]),
    Matrix(Mat4),
    Transform(&'input str, Transform),
    Scale([f32; 3]),
    Translate([f32; 3]),
    Lookat {
        target: [f32; 3],
        origin: [f32; 3],
        up: [f32; 3],
    },
    Rotate([f32; 3], f32),
    Sampler(Sampler),
    Film(Film),
    BSDF(&'input str, BSDF),
    Shape(&'input str, Shape<'input>),
    RGB(&'input str, RGBf32),
    Ref(&'input str),
    AreaEmitter(AreaEmitter),
    SunksyEmitter(SunskyEmitter),
}

#[derive(Debug)]
struct SunskyEmitter {
    turbidity: f32,
    latitude: f32,
    longitude: f32,
    timezone: f32,
    year: i32,
    month: i32,
    day: i32,
    hour: f32,
    minute: f32,
    second: f32,
    sun_direction: Option<Vec3Object>,
    sun_scale: f32,
    sky_scale: f32,
    sun_aperture: f32,
    to_world: Transform,
}

impl Default for SunskyEmitter {
    fn default() -> Self {
        Self {
            turbidity: 3.0,
            latitude: 35.689,
            longitude: 139.6917,
            timezone: 9.0,
            year: 2010,
            month: 7,
            day: 10,
            hour: 15.0,
            minute: 0.0,
            second: 0.0,
            sun_direction: None,
            sun_scale: 1.0,
            sky_scale: 1.0,
            sun_aperture: 0.5338,
            to_world: Transform::identity(),
        }
    }
}

#[derive(Debug)]
struct AreaEmitter {
    rgb: RGBf32,
}

impl Default for AreaEmitter {
    fn default() -> Self {
        Self { rgb: RGBf32::ONE }
    }
}

#[derive(Debug)]
struct Shape<'input> {
    transform: Transform,
    emitter: Option<AreaEmitter>,
    properties: Vec<String>,
    description: MeshDescription<'input>,
}

impl Default for Shape<'_> {
    fn default() -> Self {
        Self {
            transform: Transform::identity(),
            description: Default::default(),
            emitter: None,
            properties: Vec::default(),
        }
    }
}

#[derive(Debug)]
enum MeshDescription<'input> {
    TriangulatedMesh(TriangulatedMesh),
    ObjMesh(ObjMesh<'input>),
    PlyMesh(PlyMesh<'input>),
    Serialized(Serialized<'input>),
}

#[derive(Debug)]
struct Serialized<'input> {
    filename: &'input str,
    shape_index: i32,
}

impl Default for Serialized<'_> {
    fn default() -> Self {
        Self {
            filename: "",
            shape_index: 0,
        }
    }
}

impl Default for MeshDescription<'_> {
    fn default() -> Self {
        Self::TriangulatedMesh(Default::default())
    }
}

#[derive(Debug)]
struct PlyMesh<'input> {
    filename: &'input str,
    face_normals: bool,
}

impl Default for PlyMesh<'_> {
    fn default() -> Self {
        Self {
            filename: Default::default(),
            face_normals: false,
        }
    }
}

#[derive(Debug)]
struct ObjMesh<'input> {
    filename: &'input str,
    face_normals: bool,
}

impl Default for ObjMesh<'_> {
    fn default() -> Self {
        Self {
            filename: Default::default(),
            face_normals: false,
        }
    }
}

#[derive(Debug)]
struct TriangulatedMesh {
    triangles: Box<[[u32; 3]]>,
    points: Box<[Point3Object]>,
    normals: Box<[Vec3Object]>,
}

impl Default for TriangulatedMesh {
    fn default() -> Self {
        Self {
            triangles: Default::default(),
            points: Default::default(),
            normals: Default::default(),
        }
    }
}

#[derive(Debug)]
enum BSDF {
    Diffuse(Diffuse),
    Conductor(Conductor),
    Dielectric(Dielectric),
}
impl Default for BSDF {
    fn default() -> Self {
        Self::Diffuse(Diffuse::default())
    }
}

#[derive(Debug)]
struct Diffuse {
    reflectance: RGBf32,
}

impl Default for Diffuse {
    fn default() -> Self {
        Self {
            reflectance: RGBf32::splat(0.5),
        }
    }
}

#[derive(Debug)]
struct Dielectric {
    ext_ior: f32,
    int_ior: f32,
}

impl Default for Dielectric {
    fn default() -> Self {
        Self {
            ext_ior: 1.0,
            int_ior: 1.5,
        }
    }
}

#[derive(Debug)]
struct Conductor {
    roughness: f32,
    eta: RGBf32,
    k: RGBf32,
}
impl Default for Conductor {
    fn default() -> Self {
        Self {
            roughness: 0.1,
            eta: RGBf32::new(0.200438, 0.924033, 1.10221),
            k: RGBf32::new(3.91295, 2.45285, 2.14219),
        }
    }
}

#[derive(Debug)]
struct Film {
    width: i32,
    height: i32,
}

impl Default for Film {
    fn default() -> Self {
        Self {
            width: 128,
            height: 128,
        }
    }
}

#[derive(Debug)]
struct Sampler {
    sample_count: i32,
}

impl Default for Sampler {
    fn default() -> Self {
        Self {
            sample_count: Default::default(),
        }
    }
}

#[derive(Debug)]
struct Sensor {
    fov: f32,
    to_world: Transform,
    sampler: Sampler,
    film: Film,
}

impl Default for Sensor {
    fn default() -> Self {
        Self {
            fov: 30.0,
            to_world: Transform::identity(),
            sampler: Sampler::default(),
            film: Film::default(),
        }
    }
}

#[derive(Debug)]
enum Integrator {
    Path(PathIntegrator),
    LightPath(LightPathIntegrator),
    PathGuiding(PathGuidingIntegrator),
}

impl Default for Integrator {
    fn default() -> Self {
        Self::Path(Default::default())
    }
}

impl Default for PathGuidingIntegrator {
    fn default() -> Self {
        Self {
            max_depth: 16,
            spatial_threshold: 1000.0,
            directional_threshold: 0.1,
            mode: PathIntegratorMode::MIS,
        }
    }
}

#[derive(Debug)]
struct PathGuidingIntegrator {
    max_depth: usize,
    spatial_threshold: f32,
    directional_threshold: f32,
    mode: PathIntegratorMode,
}

#[derive(Debug)]
struct LightPathIntegrator {
    max_depth: usize,
}

impl Default for LightPathIntegrator {
    fn default() -> Self {
        Self {
            max_depth: usize::MAX,
        }
    }
}

#[derive(Debug)]
struct PathIntegrator {
    max_depth: usize,
    mode: PathIntegratorMode,
}

#[derive(Debug)]
enum PathIntegratorMode {
    PT,
    NEE,
    MIS,
}

impl PathIntegratorMode {
    fn io_bridge(self) -> io_bridge::PathIntegratorMode {
        match self {
            PathIntegratorMode::PT => io_bridge::PathIntegratorMode::PT,
            PathIntegratorMode::NEE => io_bridge::PathIntegratorMode::NEE,
            PathIntegratorMode::MIS => io_bridge::PathIntegratorMode::MIS,
        }
    }
}

impl Default for PathIntegrator {
    fn default() -> Self {
        Self {
            max_depth: usize::MAX,
            mode: PathIntegratorMode::MIS,
        }
    }
}

#[track_caller]
fn unused(node: roxmltree::Node) {
    println!(
        "{:}: unused node: {:?}",
        std::panic::Location::caller(),
        node
    )
}

fn attribute<'input>(
    node: roxmltree::Node<'input, 'input>,
    attr: &'input str,
    defaults: &Defaults<'_, 'input>,
) -> &'input str {
    let Some(found_attr) = attribute_checked(node, attr, defaults) else {
        panic!("attr not found: {attr} in node: {}", node.tag_name().name())
    };
    return found_attr;
}

fn attribute_checked<'input>(
    node: roxmltree::Node<'input, 'input>,
    attr: &'input str,
    defaults: &Defaults<'_, 'input>,
) -> Option<&'input str> {
    let Some(attr) = node.attribute(attr) else {
        return None;
    };
    Some(if attr.starts_with("$") {
        defaults[&attr[1..]]
    } else {
        attr
    })
}

fn children<'a, 'b>(
    node: roxmltree::Node<'a, 'b>,
) -> impl Iterator<Item = roxmltree::Node<'a, 'b>> {
    node.children().filter(|c| c.is_element())
}

fn first_child<'a, 'b>(node: roxmltree::Node<'a, 'b>) -> roxmltree::Node<'a, 'b> {
    children(node).nth(0).unwrap()
}

fn find_file(root: &std::path::Path) -> std::fs::File {
    let attempt1 = root.join("scene_v3.xml");
    std::fs::File::open(&attempt1).expect(&format!("File not found: {:?}", attempt1))
}

fn parse_ply(data: &[u8]) -> (Box<[Point3Object]>, Box<[Vec3Object]>, Box<[[u32; 3]]>) {
    struct VERTEX {
        p: Point3Object,
        n: Vec3Object,
    }
    impl ply_rs::ply::PropertyAccess for VERTEX {
        fn new() -> Self {
            Self {
                p: Default::default(),
                n: Default::default(),
            }
        }
        fn set_property(&mut self, key: String, property: ply_rs::ply::Property) {
            match (key.as_ref(), property) {
                ("x", ply_rs::ply::Property::Float(v)) => self.p.x = v,
                ("y", ply_rs::ply::Property::Float(v)) => self.p.y = v,
                ("z", ply_rs::ply::Property::Float(v)) => self.p.z = v,
                ("nx", ply_rs::ply::Property::Float(v)) => self.n.x = v,
                ("ny", ply_rs::ply::Property::Float(v)) => self.n.y = v,
                ("nz", ply_rs::ply::Property::Float(v)) => self.n.z = v,
                (k, _) => panic!("Vertex: Unexpected key/value combination: key: {}", k),
            }
        }
    }
    struct FACE(Vec<u32>);
    impl ply_rs::ply::PropertyAccess for FACE {
        fn new() -> Self {
            Self(Default::default())
        }
        fn set_property(&mut self, key: String, property: ply_rs::ply::Property) {
            match (key.as_ref(), property) {
                ("vertex_index" | "vertex_indices", ply_rs::ply::Property::ListInt(vec)) => {
                    self.0 = vec.into_iter().map(|i| i as u32).collect()
                }
                ("vertex_index" | "vertex_indices", ply_rs::ply::Property::ListUInt(vec)) => {
                    self.0 = vec.into_iter().map(|i| i as u32).collect()
                }
                (k, _) => panic!("Face: Unexpected key/value combination: key: {}", k),
            }
        }
    }

    let mut f = std::io::BufReader::new(data);
    let vertex_parser = ply_rs::parser::Parser::<VERTEX>::new();
    let face_parser = ply_rs::parser::Parser::<FACE>::new();
    let mut vertex_list = Vec::new();
    let mut face_list = Vec::new();

    let header = vertex_parser.read_header(&mut f).unwrap();
    for (_, element) in &header.elements {
        match element.name.as_ref() {
            "vertex" => {
                vertex_list = vertex_parser
                    .read_payload_for_element(&mut f, &element, &header)
                    .unwrap();
            }
            "face" => {
                face_list = face_parser
                    .read_payload_for_element(&mut f, &element, &header)
                    .unwrap();
            }
            _ => panic!("Enexpeced element!"),
        }
    }

    let (points, mut normals): (Vec<_>, Vec<_>) = vertex_list
        .into_iter()
        .map(|VERTEX { p, n }| (p, n))
        .unzip();
    if normals[0] == Vec3Object::default() {
        normals = Vec::new();
    }
    (
        points.into_boxed_slice(),
        normals.into_boxed_slice(),
        face_list
            .into_iter()
            .map(|face| match face.0.as_slice() {
                &[i, j, k] => [i as u32, j as u32, k as u32],
                _ => panic!("unknown face: {:?}", face.0.as_slice()),
            })
            .collect(),
    )
}

fn parse_obj(
    filepath: std::path::PathBuf,
) -> (Box<[Point3Object]>, Box<[Vec3Object]>, Box<[[u32; 3]]>) {
    let obj_data = obj::Obj::load(filepath).unwrap().data;
    assert_eq!(obj_data.objects.len(), 1);
    let object = &obj_data.objects[0];
    assert_eq!(object.groups.len(), 1);
    let group = &object.groups[0];
    let mut triangles = Vec::new();
    let make_tri = |poly: [obj::IndexTuple; 3]| {
        let mut tri = [poly[0].0 as u32, poly[1].0 as u32, poly[2].0 as u32];
        if let [Some(i), Some(j), Some(k)] = [poly[0].1, poly[1].1, poly[2].1] {
            assert_eq!(tri.map(|i| i as usize), [i, j, k]);
            let [n0, n1, n2] = [
                Vec3Object::from(obj_data.normal[i]),
                Vec3Object::from(obj_data.normal[j]),
                Vec3Object::from(obj_data.normal[k]),
            ];
            let ns = n0 + n1 + n2;
            let [p0, p1, p2] = [
                obj_data.position[i as usize].into(),
                obj_data.position[j as usize].into(),
                obj_data.position[k as usize].into(),
            ];
            let ng = triangle_cross(p0, p1, p2);
            if ng.dot(ns) < 0.0 {
                tri.reverse();
            }
        }
        tri
    };
    for poly in group.polys.iter() {
        if let &[i, j, k] = poly.0.as_slice() {
            let tri = make_tri([i, j, k]);
            triangles.push(tri);
        } else {
            panic!("bad length: {}", poly.0.len());
        }
    }
    assert!(obj_data.normal.len() == obj_data.position.len() || obj_data.normal.len() == 0);
    (
        obj_data.position.into_iter().map(Into::into).collect(),
        obj_data.normal.into_iter().map(Into::into).collect(),
        triangles.into_boxed_slice(),
    )
}

fn parse_serialized(
    data: &[u8],
    shape_index: usize,
) -> (Box<[Point3Object]>, Box<[Vec3Object]>, Box<[[u32; 3]]>) {
    let version = u16::from_le_bytes([data[2], data[3]]) as usize;
    const INCLUDES_VERTEX_NORMALS: u32 = 0x0001;
    const INCLUDES_TEXTURE_COORDS: u32 = 0x0002;
    const INCLUDES_VERTEX_COLORS: u32 = 0x0008;
    const USE_FACE_NORMALS: u32 = 0x0010;
    // const IS_SINGLE_PRECISION: u32 = 0x1000;
    const IS_DOUBLE_PRECISION: u32 = 0x2000;
    let offset = get_offset(data, version, shape_index) + 2 * size_of::<u16>();
    let mut stream = flate2::read::ZlibDecoder::new(&data[offset..]);
    let mask = u32::from_le_bytes(get_n(&mut stream));

    let has_normals = mask & INCLUDES_VERTEX_NORMALS != 0;
    let has_texcoords = mask & INCLUDES_TEXTURE_COORDS != 0;
    let has_colors = mask & INCLUDES_VERTEX_COLORS != 0;
    let _face_normals = mask & USE_FACE_NORMALS != 0;
    // let is_single = mask & IS_SINGLE_PRECISION != 0;
    let is_double = mask & IS_DOUBLE_PRECISION != 0;
    assert!(!is_double);
    let n_vertices = u64::from_le_bytes(get_n(&mut stream)) as usize;
    let n_triangles = u64::from_le_bytes(get_n(&mut stream)) as usize;
    // dbg!(
    //     [n_triangles, n_vertices],
    //     [has_normals, has_texcoords, has_colors, face_normals]
    // );

    let mut buffer_n_triangles_3xu32 = vec![0u8; n_triangles * size_of::<[u32; 3]>()];
    let mut buffer_n_vertices_3xf32 = vec![0u8; n_vertices * size_of::<[f32; 3]>()];
    let mut buffer_n_vertices_2xf32 = vec![0u8; n_vertices * size_of::<[f32; 2]>()];

    let mut positions = vec![Point3Object::ZERO; n_vertices];
    {
        let buffer = &mut buffer_n_vertices_3xf32;
        stream.read_exact(buffer).unwrap();
        for (p, &arr) in positions.iter_mut().zip(buffer.array_chunks()) {
            *p = Point3Object::from_le_bytes(arr);
        }
    }

    let mut normals = vec![Vec3Object::ZERO; if has_normals { n_vertices } else { 0 }];
    if has_normals {
        let buffer = &mut buffer_n_vertices_3xf32;
        stream.read_exact(buffer).unwrap();
        for (p, &arr) in normals.iter_mut().zip(buffer.array_chunks()) {
            *p = Vec3Object::from_le_bytes(arr);
        }
    }

    let mut tex_coords = vec![Point2::ZERO; if has_texcoords { n_vertices } else { 0 }];
    if has_texcoords {
        let buffer = &mut buffer_n_vertices_2xf32;
        stream.read_exact(buffer).unwrap();
        for (p, &arr) in tex_coords.iter_mut().zip(buffer.array_chunks()) {
            *p = Point2::from_le_bytes(arr);
        }
    }

    let mut colors = vec![RGBf32::ZERO; if has_colors { n_vertices } else { 0 }];
    if has_colors {
        let buffer = &mut buffer_n_vertices_3xf32;
        stream.read_exact(buffer).unwrap();
        for (p, &arr) in colors.iter_mut().zip(buffer.array_chunks()) {
            *p = RGBf32::from_le_bytes(arr);
        }
    }

    let mut triangles = vec![[0u32; 3]; n_triangles];
    {
        let buffer = &mut buffer_n_triangles_3xu32;
        stream.read_exact(buffer).unwrap();
        for (tri, &[a, b, c, d, e, f, g, h, i, j, k, l]) in
            triangles.iter_mut().zip(buffer.array_chunks())
        {
            *tri = [
                u32::from_le_bytes([a, b, c, d]),
                u32::from_le_bytes([e, f, g, h]),
                u32::from_le_bytes([i, j, k, l]),
            ];
        }
    }

    (
        positions.into_boxed_slice(),
        normals.into_boxed_slice(),
        triangles.into_boxed_slice(),
    )
}

fn get_offset(data: &[u8], version: usize, shape_index: usize) -> usize {
    let n_shapes = u32::from_le_bytes([
        data[data.len() - 4],
        data[data.len() - 3],
        data[data.len() - 2],
        data[data.len() - 1],
    ]) as usize;
    assert!(shape_index <= shape_index);
    assert_eq!(version, 3);
    let index = data.len() - size_of::<u32>() * (n_shapes - shape_index + 1);
    u32::from_le_bytes([
        data[index + 0],
        data[index + 1],
        data[index + 2],
        data[index + 3],
    ]) as usize
}
fn get_n<const N: usize>(data: &mut impl std::io::Read) -> [u8; N] {
    let mut res = [0; N];
    data.read_exact(&mut res).unwrap();
    res
}
