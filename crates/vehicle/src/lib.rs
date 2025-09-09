// Everything prefixed with `JPC_` comes from the joltc_sys crate.
use joltc_sys::*;
use std::ffi::{CStr, CString};
use std::ptr;

pub fn create_box(settings: &JPC_BoxShapeSettings) -> Result<*mut JPC_Shape, CString> {
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();

    unsafe {
        if JPC_BoxShapeSettings_Create(settings, &mut shape, &mut err) {
            Ok(shape)
        } else {
            Err(CStr::from_ptr(JPC_String_c_str(err)).to_owned())
        }
    }
}

pub fn create_cylinder(settings: &JPC_CylinderShapeSettings) -> Result<*mut JPC_Shape, CString> {
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();

    unsafe {
        if JPC_CylinderShapeSettings_Create(settings, &mut shape, &mut err) {
            Ok(shape)
        } else {
            Err(CStr::from_ptr(JPC_String_c_str(err)).to_owned())
        }
    }
}

pub fn create_sphere(settings: &JPC_SphereShapeSettings) -> Result<*mut JPC_Shape, CString> {
    let mut shape: *mut JPC_Shape = ptr::null_mut();
    let mut err: *mut JPC_String = ptr::null_mut();

    unsafe {
        if JPC_SphereShapeSettings_Create(settings, &mut shape, &mut err) {
            Ok(shape)
        } else {
            Err(CStr::from_ptr(JPC_String_c_str(err)).to_owned())
        }
    }
}

pub fn vec3(x: f32, y: f32, z: f32) -> JPC_Vec3 {
    JPC_Vec3 { x, y, z, _w: z }
}

pub fn rvec3(x: Real, y: Real, z: Real) -> JPC_RVec3 {
    JPC_RVec3 { x, y, z, _w: z }
}

pub fn vec4(x: f32, y: f32, z: f32, w: f32) -> JPC_Vec4 {
    JPC_Vec4 { x, y, z, w }
}

// If 'double-precision' is set, there is padding in this struct
#[allow(clippy::needless_update)]
pub fn rmat44_identity() -> JPC_RMat44 {
    unsafe {
        JPC_RMat44 {
            col: [
                vec4(1.0, 0.0, 0.0, 0.0),
                vec4(0.0, 1.0, 0.0, 0.0),
                vec4(0.0, 0.0, 1.0, 0.0),
            ],
            col3: rvec3(0.0, 0.0, 0.0),
            ..std::mem::zeroed()
        }
    }
}

pub fn rmat44_translation(col3: JPC_RVec3) -> JPC_RMat44 {
    JPC_RMat44 {
        col3,
        ..rmat44_identity()
    }
}
