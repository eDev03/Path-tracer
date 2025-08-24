use std::sync::Mutex;

use crate::geom::*;
use crate::io_bridge;
use crate::sg;
use crate::spaces::*;
use crate::spectrum::SampledSpectrum;
use crate::spectrum::SampledWavelengths;
use crate::spectrum::{rgb_to_xyz, xyz_to_rgb};

use math::*;

pub struct Camera {
    pub pixel_size: f32,
    pub image_size_f32: [f32; 2],
    pub world_to_camera: Matrix4f32<Worldspace, Cameraspace>,
    pub camera_to_world: Matrix4f32<Cameraspace, Worldspace>,
}

pub struct CameraWeSample {
    pub ray: Ray,
    pub pdf: f32,
}

pub struct CameraWiSample {
    pub importance: SampledSpectrum,
    pub wc: Vec3World,
    pub pdf: f32,
    pub p_raster: Point2Raster,
}

impl Camera {
    pub fn new(opt: &io_bridge::CameraOptions) -> Self {
        let pixel_size = 2.0 * ((opt.fov / 2.0).to_radians().tan()) / opt.image_size[1] as f32;
        Self {
            image_size_f32: [opt.image_size[0] as f32, opt.image_size[1] as f32],
            pixel_size,
            world_to_camera: opt.world_to_camera,
            camera_to_world: opt.world_to_camera.inverse(),
        }
    }
    pub fn p(&self) -> Point3World {
        self.camera_to_world * Point3Camera::ZERO
    }
    pub fn n(&self) -> Vec3World {
        (self.camera_to_world * Vec3Camera::new(0.0, 0.0, 1.0)).normalized()
    }
    pub fn r(&self) -> Vec3World {
        (self.camera_to_world * Vec3Camera::new(1.0, 0.0, 0.0)).normalized()
    }
    pub fn sample_we(&self, ix: usize, iy: usize, u2d: [f32; 2]) -> CameraWeSample {
        let (x, y) = (
            (self.image_size_f32[0] * 0.5 - ix as f32 - u2d[0]) * self.pixel_size,
            (self.image_size_f32[1] * 0.5 - iy as f32 - u2d[1]) * self.pixel_size,
        );
        CameraWeSample {
            ray: Ray {
                o: self.camera_to_world * Point3Camera::new(0.0, 0.0, 0.0),
                d: (self.camera_to_world * Vec3Camera::new(x, y, 1.0)).normalized(),
            },
            pdf: 1.0,
        }
    }
    pub fn sample_wi(&self, from_p: Point3World) -> Option<CameraWiSample> {
        let to_cam = self.p() - from_p;
        let wc = to_cam.normalized();
        let cam_space_dir = self.world_to_camera * -wc;
        let p_raster = self.dir_to_p_raster(cam_space_dir)?;
        let dist2 = to_cam.mag_sq();
        let cos_theta = self.n().dot(wc).abs();
        let pdf = dist2 / cos_theta;
        Some(CameraWiSample {
            importance: self.eval_we(cam_space_dir),
            wc,
            pdf,
            p_raster,
        })
    }
    pub fn eval_we(&self, dir: Vec3Camera) -> SampledSpectrum {
        SampledSpectrum::splat(
            1.0 / (self.pixel_size
                * self.pixel_size
                * self.image_size_f32[0]
                * self.image_size_f32[1]
                * sg::cos_theta(dir).powi(4)),
        )
    }
    pub fn pdf_we(&self, dir: Vec3Camera) -> (f32, f32) {
        let Some(_p_raster) = self.dir_to_p_raster(dir) else {
            return (0.0, 0.0);
        };
        let area =
            self.pixel_size * self.pixel_size * self.image_size_f32[0] * self.image_size_f32[1];
        let cos_theta = sg::cos_theta(dir);
        (1.0 / area, 1.0 / (area * cos_theta.powi(3)))
    }
    fn dir_to_p_raster(&self, dir: Vec3Camera) -> Option<Point2Raster> {
        if dir.z <= 0.0 {
            return None;
        }
        let p = self.dir_to_p_raster_unchecked(dir);
        if self.contains(p) { Some(p) } else { None }
    }
    fn dir_to_p_raster_unchecked(&self, dir: Vec3Camera) -> Point2Raster {
        let dir = dir / dir.z;
        let x = self.image_size_f32[0] / 2.0 - dir.x / self.pixel_size;
        let y = self.image_size_f32[1] / 2.0 - dir.y / self.pixel_size;
        return Point2f32::new(x, y);
    }
    fn contains(&self, p: Point2Raster) -> bool {
        (0.0 <= p.x && p.x <= self.image_size_f32[0])
            && (0.0 <= p.y && p.y <= self.image_size_f32[1])
    }
}

pub struct Film {
    pub tiles: Box<[FilmTile]>,
    pub splat_pixels: Box<[SplatPixel]>,
    pub image_size: [usize; 2],
}
pub struct FilmTile {
    sample_pixels: Box<[SamplePixel]>,
    min: [usize; 2],
    size: [usize; 2],
}
#[derive(Debug, Default, Copy, Clone)]
pub struct SamplePixel {
    rgb: RGBf64,
}
#[derive(Debug, Default)]
pub struct SplatPixel {
    rgb: Mutex<RGBf64>,
}

unsafe impl Send for FilmTile {}

impl SamplePixel {
    pub fn add(&mut self, l: &SampledSpectrum, swl: &SampledWavelengths) {
        self.rgb += xyz_to_rgb(l.to_xyz(swl));
    }
}
impl SplatPixel {
    pub fn add(&self, l: &SampledSpectrum, swl: &SampledWavelengths) {
        let xyz = l.to_xyz(swl);
        *self.rgb.lock().unwrap() += xyz_to_rgb(xyz);
    }
}

impl Film {
    pub fn new(opt: &io_bridge::CameraOptions) -> Self {
        Self {
            tiles: Self::create_tiles(opt.image_size, 16).into_boxed_slice(),
            splat_pixels: (0..(opt.image_size[0] * opt.image_size[1]))
                .into_iter()
                .map(|_| SplatPixel::default())
                .collect(),

            image_size: opt.image_size,
        }
    }
    pub fn add_splat(&self, p_raster: Point2Raster, l: &SampledSpectrum, swl: &SampledWavelengths) {
        let ix = (p_raster.x as usize).min(self.image_size[0] - 1);
        let iy = (p_raster.y as usize).min(self.image_size[1] - 1);
        let idx = ix + iy * self.image_size[0];
        self.splat_pixels[idx].add(l, swl);
    }
    fn create_tiles(image_size: [usize; 2], tile_size: usize) -> Vec<FilmTile> {
        let mut tiles = vec![];
        for y in 0..image_size[1].div_ceil(tile_size) {
            for x in 0..image_size[0].div_ceil(tile_size) {
                let min = [x * tile_size, y * tile_size];
                let remaining = [image_size[0] - min[0], image_size[1] - min[1]];
                let size = [tile_size.min(remaining[0]), tile_size.min(remaining[1])];
                tiles.push(FilmTile {
                    sample_pixels: vec![SamplePixel::default(); size[0] * size[1]]
                        .into_boxed_slice(),
                    min,
                    size,
                });
            }
        }
        tiles
    }
    pub fn create_pixels(&self, scale: f64) -> Box<[RGBf64]> {
        let mut image = vec![RGBf64::ZERO; self.image_size[0] * self.image_size[1]];
        for (img_p, p) in image.iter_mut().zip(self.splat_pixels.iter()) {
            *img_p += p.rgb.get_cloned().unwrap() * scale;
        }
        for tile in self.tiles.iter() {
            for (p, tile_index) in tile.iter() {
                let film_index = tile.film_index(tile_index);
                image[film_index[0] + film_index[1] * self.image_size[0]] += p.rgb * scale;
            }
        }
        image.into()
    }
    pub fn save_image(&self, scale: f64, file_name: &str) {
        let mut data = vec![0; self.image_size[0] * self.image_size[1] * 4];

        for ([r, g, b, a], p) in data.array_chunks_mut().zip(self.splat_pixels.iter()) {
            let rgb = p.rgb.get_cloned().unwrap() * scale;
            *r = (rgb[0].powf(1.0 / 2.2).clamp(0.0, 1.0) * 255.999) as u8;
            *g = (rgb[1].powf(1.0 / 2.2).clamp(0.0, 1.0) * 255.999) as u8;
            *b = (rgb[2].powf(1.0 / 2.2).clamp(0.0, 1.0) * 255.999) as u8;
            *a = 255;
        }
        for tile in self.tiles.iter() {
            for (p, tile_index) in tile.iter() {
                let rgb = p.rgb * scale;
                // let mut xyz = rgb_to_xyz(rgb);
                // let y_in = xyz.y;
                // let y_out = y_in / (1.0 + y_in);
                // let rgb = rgb * y_out / y_in;
                // dbg!(rgb.y);
                let film_index = tile.film_index(tile_index);
                data[4 * (film_index[0] + film_index[1] * self.image_size[0]) + 0] +=
                    (rgb[0].powf(1.0 / 2.4).clamp(0.0, 1.0) * 255.999) as u8;
                data[4 * (film_index[0] + film_index[1] * self.image_size[0]) + 1] +=
                    (rgb[1].powf(1.0 / 2.4).clamp(0.0, 1.0) * 255.999) as u8;
                data[4 * (film_index[0] + film_index[1] * self.image_size[0]) + 2] +=
                    (rgb[2].powf(1.0 / 2.4).clamp(0.0, 1.0) * 255.999) as u8;
            }
        }
        let root = std::path::Path::new("zout");
        if !root.exists() {
            std::fs::create_dir(root).unwrap();
        }
        let mut writer = stb_image_write_rust::ImageWriter::ImageWriter::new(
            root.join(file_name).to_str().unwrap(),
        );
        writer.write_png(
            self.image_size[0] as i32,
            self.image_size[1] as i32,
            4,
            data.as_ptr(),
        );
    }

    pub fn clear(&mut self) {
        self.splat_pixels
            .iter_mut()
            .for_each(|pixel| *pixel.rgb.get_mut().unwrap() = RGBf64::ZERO);
        self.tiles.iter_mut().for_each(|tile| {
            tile.iter_mut()
                .for_each(|(pixel, _)| pixel.rgb = RGBf64::ZERO)
        });
    }
}

impl FilmTile {
    pub fn offset(&self) -> [usize; 2] {
        self.min
    }
    pub fn iter(&self) -> TileIter {
        TileIter {
            internal: self.sample_pixels.iter(),
            tile_size: self.size,
            tile_index: [0; 2],
        }
    }
    pub fn iter_mut(&mut self) -> TileIterMut {
        TileIterMut {
            internal: self.sample_pixels.iter_mut(),
            tile_size: self.size,
            tile_index: [0; 2],
        }
    }
    pub fn film_index(&self, index: [usize; 2]) -> [usize; 2] {
        let x = self.min[0] + index[0];
        let y = self.min[1] + index[1];
        [x, y]
    }
}
pub struct TileIter<'tile> {
    internal: core::slice::Iter<'tile, SamplePixel>,
    tile_size: [usize; 2],
    tile_index: [usize; 2],
}
pub struct TileIterMut<'tile> {
    internal: core::slice::IterMut<'tile, SamplePixel>,
    tile_size: [usize; 2],
    tile_index: [usize; 2],
}

impl<'tile> Iterator for TileIter<'tile> {
    type Item = (&'tile SamplePixel, [usize; 2]);

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.tile_index;
        let pixel = self.internal.next()?;
        self.tile_index[0] += 1;
        if self.tile_index[0] >= self.tile_size[0] {
            self.tile_index[0] = 0;
            self.tile_index[1] += 1;
        }

        return Some((pixel, index));
    }
}

impl<'tile> Iterator for TileIterMut<'tile> {
    type Item = (&'tile mut SamplePixel, [usize; 2]);

    fn next(&mut self) -> Option<Self::Item> {
        let index = self.tile_index;
        let pixel = self.internal.next()?;
        self.tile_index[0] += 1;
        if self.tile_index[0] >= self.tile_size[0] {
            self.tile_index[0] = 0;
            self.tile_index[1] += 1;
        }

        return Some((pixel, index));
    }
}
