#![feature(array_chunks)]

use itertools::Itertools;
use math::*;
use sky::*;

use beryllium::*;
use ogl33::*;

fn main() {
    let sdl = Sdl::init(init::InitFlags::EVERYTHING);
    sdl.set_gl_context_major_version(3).unwrap();
    sdl.set_gl_context_minor_version(3).unwrap();
    let win_args = video::CreateWinArgs {
        title: "title",
        width: 800,
        height: 600,
        allow_high_dpi: true,
        borderless: false,
        resizable: false,
    };
    let vertices: [[f32; 4]; 6] = [
        [-1.0, -1.0, 0.0, 0.0],
        [1.0, -1.0, 1.0, 0.0],
        [1.0, 1.0, 1.0, 1.0],
        [1.0, 1.0, 1.0, 1.0],
        [-1.0, 1.0, 0.0, 1.0],
        [-1.0, -1.0, 0.0, 0.0],
    ];
    let win = sdl.create_gl_window(win_args).unwrap();
    unsafe {
        load_gl_with(|f_name| win.get_proc_address(f_name.cast()));
    }
    let (vao, vbo) = unsafe {
        let mut vao = 0;
        glGenVertexArrays(1, &mut vao);
        assert_ne!(vao, 0);
        let mut vbo = 0;
        glGenBuffers(1, &mut vbo);
        assert_ne!(vbo, 0);
        (vao, vbo)
    };
    unsafe {
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(
            GL_ARRAY_BUFFER,
            size_of_val(&vertices) as isize,
            vertices.as_ptr().cast(),
            GL_STATIC_DRAW,
        );
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 16, 0 as *const _);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 16, 8 as *const _);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
    }
    let program = unsafe {
        let vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        let fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
        assert_ne!(vertex_shader, 0);
        assert_ne!(fragment_shader, 0);
        const VERTEX_SHADER: &str = r#"
        #version 330 core
        layout (location = 0) in vec2 pos;
        layout (location = 1) in vec2 uvs;

        out vec2 uv;

        void main() {
            uv = uvs;
            gl_Position = vec4(pos.x, pos.y, 0.0, 1.0);
        }"#;
        const FRAGMENT_SHADER: &str = include_str!("frag.glsl");
        glShaderSource(
            vertex_shader,
            1,
            &(VERTEX_SHADER.as_bytes().as_ptr().cast()),
            &(VERTEX_SHADER.len().try_into().unwrap()),
        );
        glCompileShader(vertex_shader);
        let mut success = 0;
        glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &mut success);
        if success == 0 {
            let mut v = [0u8; 1024];
            let mut log_len = 0;
            glGetShaderInfoLog(
                vertex_shader,
                v.len() as i32,
                &mut log_len,
                v.as_mut_ptr().cast(),
            );
            panic!(
                "Vertex Compile Error: {}",
                String::from_utf8_lossy(&v[..log_len as usize])
            );
        }
        glShaderSource(
            fragment_shader,
            1,
            &(FRAGMENT_SHADER.as_bytes().as_ptr().cast()),
            &(FRAGMENT_SHADER.len().try_into().unwrap()),
        );
        glCompileShader(fragment_shader);
        let mut success = 0;
        glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &mut success);
        if success == 0 {
            let mut v = [0u8; 1024];
            let mut log_len = 0;
            glGetShaderInfoLog(
                fragment_shader,
                v.len() as i32,
                &mut log_len,
                v.as_mut_ptr().cast(),
            );
            panic!(
                "Fragment Compile Error: {}",
                String::from_utf8_lossy(&v[..log_len as usize])
            );
        }
        let program = glCreateProgram();
        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);
        glLinkProgram(program);
        let mut success = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &mut success);
        if success == 0 {
            let mut v = [0u8; 1024];
            let mut log_len = 0;
            glGetProgramInfoLog(program, v.len() as i32, &mut log_len, v.as_mut_ptr().cast());
            panic!(
                "Program Link Error: {}",
                String::from_utf8_lossy(&v[..log_len as usize])
            );
        }
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        program
    };
    unsafe {
        glUseProgram(program);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
    }
    'main_loop: loop {
        while let Some(event) = sdl.poll_events() {
            match event {
                (events::Event::Quit, _) => break 'main_loop,
                _ => {}
            }
        }
        unsafe {
            glClear(GL_COLOR_BUFFER_BIT);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }
        win.swap_window();
    }
}

fn compute_image(width: usize, height: usize, sun_dir: Vec3Skybox, turbidity: f32) {
    let theta_s = acos(sun_dir.y);
    let model = hosek::create_rgb_model(turbidity, 0.5, PI_OVER_TWO - theta_s, 4.0);
    let img = (0..height)
        .cartesian_product(0..width)
        .map(|(h, w)| {
            let u = (w as f32 / width as f32) * 0.5;
            let v = h as f32 / height as f32;
            let view_dir = uv_to_dir([u, v]);
            let theta = acos(view_dir.y);
            let gamma = acos(view_dir.dot(sun_dir));
            let sky = hosek::sky_radiance(&model, theta, gamma);
            let sun = hosek::sun_radiance(&model, theta, gamma);

            [(sky[0] + sun[0]), (sky[1] + sun[1]), (sky[2] + sun[2])]
        })
        .collect::<Box<[[f32; 3]]>>();
    let data = img
        .iter()
        .flat_map(|rgb| {
            let [r, g, b] = rgb;
            [
                (r.clamp(0.0, 1.0).powf(1.0 / 2.4) * 255.999) as u8,
                (g.clamp(0.0, 1.0).powf(1.0 / 2.4) * 255.999) as u8,
                (b.clamp(0.0, 1.0).powf(1.0 / 2.4) * 255.999) as u8,
                (255.999) as u8,
            ]
        })
        .collect_vec();
    let mut writer = stb_image_write_rust::ImageWriter::ImageWriter::new("out.png");
    writer.write_png(width as i32, height as i32, 4, data.as_ptr());
}

fn uv_to_dir([u, v]: [f32; 2]) -> Vec3Skybox {
    let phi = u * TWO_PI;
    let theta = v * PI;
    let cos_theta = theta.cos();
    let sin_theta = theta.sin();
    let cos_phi = phi.cos();
    let sin_phi = phi.sin();
    [sin_theta * cos_phi, sin_theta * sin_phi, cos_theta].into()
}
