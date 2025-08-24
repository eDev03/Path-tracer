use math::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SkyboxSpace;

pub type Vec3Skybox = Vector3f32<SkyboxSpace>;
pub type Vec3f64Skybox = Vector3f64<SkyboxSpace>;

const SUN_HALF_APERTURE: f32 = (0.5358f32 / 2.0).to_radians();

const EARTH_MEAN_RADIUS: f32 = 6371.01; // In km
const ASTRONOMICAL_UNIT: f32 = 149597890.0; // In km

/// Conversion constant to convert spectral solar luminosity to RGB
const SPEC_TO_RGB_SUN_CONV: f32 = 467.069280386;

/// latitude and longitude is in degrees
/// timezone is offset in hours
pub struct EarthLocation {
    pub latitude: f32,
    pub longitude: f32,
    pub timezone: f32,
}
pub struct DateTime {
    pub year: f32,
    pub month: f32,
    pub day: f32,
    pub hour: f32,
    pub minute: f32,
    pub second: f32,
}

/// positive x points west
/// positive y points at zenith
/// positive z points north
pub fn compute_solar_vector(location: EarthLocation, date_time: DateTime) -> Vec3Skybox {
    // based on: https://github.com/mitsuba-renderer/mitsuba/blob/master/src/emitters/sunsky/sunmodel.h

    // Main variables
    let [elapsed_julian_days, dec_hours];
    let [ecliptic_longitude, ecliptic_obliquity];
    let [mut right_ascension, declination];
    let [mut elevation, mut azimuth];

    // Auxiliary variables
    let mut d_y;
    let mut d_x;

    /* Calculate difference in days between the current Julian Day
    and JD 2451545.0, which is noon 1 January 2000 Universal Time */
    {
        // Calculate time of the day in UT decimal hours
        dec_hours = date_time.hour - location.timezone
            + (date_time.minute + date_time.second / 60.0) / 60.0;

        // Calculate current Julian Day
        let li_aux_1 = (date_time.month - 14.0) / 12.0;
        let li_aux_2 = (1461.0 * (date_time.year + 4800.0 + li_aux_1)) / 4.0
            + (367.0 * (date_time.month - 2.0 - 12.0 * li_aux_1)) / 12.0
            - (3.0 * ((date_time.year + 4900.0 + li_aux_1) / 100.0)) / 4.0
            + date_time.day
            - 3207.05;
        let d_julian_date = li_aux_2 - 0.5 + dec_hours / 24.0;

        // Calculate difference between current Julian Day and JD 2451545.0
        elapsed_julian_days = d_julian_date - 2451545.0;
    }

    /* Calculate ecliptic coordinates (ecliptic longitude and obliquity of the
    ecliptic in radians but without limiting the angle to be less than 2*Pi
    (i.e., the result may be greater than 2*Pi) */
    {
        let omega = 2.1429 - 0.0010394594 * elapsed_julian_days;
        let mean_longitude = 4.8950630 + 0.017202791698 * elapsed_julian_days; // Radians
        let anomaly = 6.2400600 + 0.0172019699 * elapsed_julian_days;

        ecliptic_longitude =
            mean_longitude + 0.03341607 * sin(anomaly) + 0.00034894 * sin(2.0 * anomaly)
                - 0.0001134
                - 0.0000203 * sin(omega);

        ecliptic_obliquity = 0.4090928 - 6.2140e-9 * elapsed_julian_days + 0.0000396 * cos(omega);
    }

    /* Calculate celestial coordinates ( right ascension and declination ) in radians
    but without limiting the angle to be less than 2*Pi (i.e., the result may be
    greater than 2*Pi) */
    {
        let sin_ecliptic_longitude = sin(ecliptic_longitude);
        d_y = cos(ecliptic_obliquity) * sin_ecliptic_longitude;
        d_x = cos(ecliptic_longitude);
        right_ascension = atan2(d_y, d_x);
        right_ascension += if right_ascension < 0.0 { TWO_PI } else { 0.0 };

        declination = asin(sin(ecliptic_obliquity) * sin_ecliptic_longitude);
    }

    // Calculate local coordinates (azimuth and zenith angle) in degrees
    {
        let greenwich_mean_sidereal_time =
            6.6974243242 + 0.0657098283 * elapsed_julian_days + dec_hours;

        let local_mean_sidereal_time =
            (greenwich_mean_sidereal_time * 15.0 + location.longitude).to_radians();

        let latitude_in_radians = (location.latitude).to_radians();
        let cos_latitude = cos(latitude_in_radians);
        let sin_latitude = sin(latitude_in_radians);

        let hour_angle = local_mean_sidereal_time - right_ascension;
        let cos_hour_angle = cos(hour_angle);

        elevation = acos(
            cos_latitude * cos_hour_angle * cos(declination) + sin(declination) * sin_latitude,
        );

        d_y = -sin(hour_angle);
        d_x = tan(declination) * cos_latitude - sin_latitude * cos_hour_angle;

        azimuth = atan2(d_y, d_x);
        azimuth += if azimuth < 0.0 { TWO_PI } else { 0.0 };

        // Parallax Correction
        elevation += (EARTH_MEAN_RADIUS / ASTRONOMICAL_UNIT) * sin(elevation);
    }

    let cos_theta = elevation.cos();
    let sin_theta = elevation.sin();
    let cos_phi = azimuth.cos();
    let sin_phi = azimuth.sin();
    Vec3Skybox::new(sin_phi * sin_theta, cos_theta, -cos_phi * sin_theta).normalized()
}
#[allow(non_snake_case)]
pub(crate) fn Yxy_to_xyz(Y: f32, x: f32, y: f32) -> [f32; 3] {
    // y is proportional to Y, avoiding division by zero
    // by returning black (Y=0) is fine
    if y == 0.0 {
        return [0.0; 3];
    }
    let X = x * (Y / y);
    let Z = (1.0 - x - y) * (Y / y);
    [X, Y, Z]
}

pub fn xyz_to_rgb(xyz: Vec3Skybox) -> Vec3Skybox {
    Vec3Skybox::from([
        xyz.dot(Vec3Skybox::from([3.240479, -1.537150, -0.498535])),
        xyz.dot(Vec3Skybox::from([-0.969256, 1.875991, 0.041556])),
        xyz.dot(Vec3Skybox::from([0.055648, -0.204043, 1.057311])),
    ])
}

fn xyz_g(x: f32, m: f32, t1: f32, t2: f32) -> f32 {
    if x < m {
        exp(-t1.powi(2) * (x - m).powi(2) / 2.0)
    } else {
        exp(-t2.powi(2) * (x - m).powi(2) / 2.0)
    }
}

pub fn xyz_x(l: f32) -> f32 {
    1.056 * xyz_g(l, 599.8, 0.0264, 0.0323) + 0.362 * xyz_g(l, 442.0, 0.0624, 0.0374)
        - 0.065 * xyz_g(l, 501.1, 0.049, 0.0382)
}

pub fn xyz_y(l: f32) -> f32 {
    0.821 * xyz_g(l, 568.8, 0.0213, 0.0247) + 0.286 * xyz_g(l, 530.9, 0.0613, 0.0322)
}

pub fn xyz_z(l: f32) -> f32 {
    1.217 * xyz_g(l, 437.0, 0.0845, 0.0278) + 0.681 * xyz_g(l, 459.0, 0.0385, 0.0725)
}
