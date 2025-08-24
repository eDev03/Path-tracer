#![feature(array_chunks)]
#![feature(iter_array_chunks)]
#![feature(type_changing_struct_update)]
#![feature(float_next_up_down)]
#![feature(let_chains)]

pub mod bsdfs;
pub mod bvh;
pub mod camera;
pub mod geom;
pub mod integrator;
pub mod io_bridge;
pub mod lighting;
pub mod parser;
pub mod sampling;
pub mod scene;
pub mod sg;
pub mod spaces;
pub mod spectrum;

use camera::*;
use integrator::*;
use scene::*;

fn main() {
    //    makes the pc usable for other things while rendering
    //    rayon::ThreadPoolBuilder::new()
    //        .num_threads(7)
    //        .build_global()
    //        .unwrap();
    for arg in std::env::args().skip(1) {
        let parse_result = parser::parse("scenes", &arg);

        let cam = Camera::new(&parse_result.camera);
        let scene = Scene::new(
            parse_result.meshes,
            parse_result.materials,
            parse_result.envmap,
        );
        let mut film = Film::new(&parse_result.camera);
        let mut renderer = Integrator::new(&parse_result.integrator);
        let start_time = std::time::Instant::now();
        renderer.render(&scene, parse_result.camera.spp, &cam, &mut film);
        dbg!(start_time.elapsed(), &parse_result.output_name);

        film.save_image(
            renderer.film_scale(parse_result.camera.spp),
            &parse_result.output_name,
        );
    }
}
