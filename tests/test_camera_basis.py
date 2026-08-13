# test_camera_basis.py
# The camera's stored basis vectors must agree with the basis actually used
# to render. They disagreed for a long time: NightCamera used
# right = cross(up, forward) while NightMatrix.get_lookat -- which builds
# the view matrix -- uses right = cross(forward, up). The rendered image was
# fine (get_lookat is self-consistent), but get_right_vector() returned the
# opposite of on-screen right, which silently mirrors anything that reasons
# about screen space, such as photometric calibration ground truth.
#
# Offline: no OpenGL context needed, since this is pure matrix maths.
#
# run:  python tests/test_camera_basis.py

import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from NightEngine.NightMatrix import NightMatrix


def basis_from_view(view):
    """the renderer's basis: get_lookat packs [right, up, -forward] as the
    rows of the rotation block."""
    return view[0, 0:3].copy(), view[1, 0:3].copy(), -view[2, 0:3].copy()


def lookat_basis(position, target, up):
    view = NightMatrix.get_lookat(position, target, up)
    return basis_from_view(view)


# ------------------------------------------------------------
# the convention itself
# ------------------------------------------------------------

def test_get_lookat_is_right_handed():
    """right x up must equal -forward, i.e. a GL-style eye basis."""
    cases = [([0, 0, 0], [0, 0, -1], [0, 1, 0]),
             ([0, 20, 0], [0, 0, 0], [0, 0, 1]),
             ([3, 4, 5], [-1, 0, 2], [0, 1, 0])]
    for position, target, up in cases:
        right, camera_up, forward = lookat_basis(position, target, up)
        assert np.allclose(np.cross(right, camera_up), -forward, atol=1e-5), (
            f"basis is not right-handed for {position}->{target}")
        for vector in (right, camera_up, forward):
            assert abs(np.linalg.norm(vector) - 1.0) < 1e-5


def test_get_lookat_right_is_forward_cross_up():
    position, target, up = [0, 20, 0], [0, 0, 0], [0, 0, 1]
    right, _, forward = lookat_basis(position, target, up)
    expected = np.cross(forward, np.array(up, dtype=np.float64))
    expected /= np.linalg.norm(expected)
    assert np.allclose(right, expected, atol=1e-5)


# ------------------------------------------------------------
# NightCamera must agree with it
# ------------------------------------------------------------

def _camera():
    """a NightCamera without touching OpenGL. NightCamera is a pure
    transform node, so constructing it needs no GL context."""
    from NightEngine.NightCamera import NightCamera
    return NightCamera()


def test_look_at_stored_basis_matches_rendered_basis():
    for position, target in [([0, 20, 0], [0, 0, 0]),          # straight down
                             ([0, 20, 0.001], [0, 0, 0]),       # near-degenerate
                             ([12, 6, 9], [0, 1, 0]),
                             ([-4, 2, -7], [3, 3, 3]),
                             ([0, -15, 0], [0, 0, 0])]:         # straight up
        camera = _camera()
        camera.look_at(position=position, target=target)
        camera.update()
        right, up, forward = basis_from_view(camera.matrix_view)

        assert np.allclose(camera.get_right_vector(), right, atol=1e-5), (
            f"stored right {camera.get_right_vector()} != rendered right "
            f"{right} for {position}->{target}")
        assert np.allclose(camera.get_up_vector(), up, atol=1e-5), (
            f"stored up != rendered up for {position}->{target}")
        assert np.allclose(camera.get_forward_vector(), forward, atol=1e-5), (
            f"stored forward != rendered forward for {position}->{target}")


def test_move_keeps_stored_basis_consistent():
    """the free-look camera recomputes its basis from yaw/pitch; that path
    must agree with the renderer too."""
    for yaw, pitch in [(-90, 0), (0, 0), (45, 30), (170, -60), (-15, 85)]:
        camera = _camera()
        camera.yaw, camera.pitch = yaw, pitch
        camera.set_position([1.0, 2.0, 3.0])
        # keep the test hermetic: move() polls glfw for key state, which
        # needs an initialised library and a window. Stub it out so we
        # exercise only the basis recomputation.
        camera.check_pressed = lambda window, key: False
        camera.move(None, 0.0)
        camera.update()
        right, up, forward = basis_from_view(camera.matrix_view)
        assert np.allclose(camera.get_right_vector(), right, atol=1e-5), (
            f"stored right != rendered right at yaw={yaw} pitch={pitch}")
        assert np.allclose(camera.get_up_vector(), up, atol=1e-5), (
            f"stored up != rendered up at yaw={yaw} pitch={pitch}")


def test_right_vector_points_screen_right():
    """concrete sanity check: looking down -Z with +Y up, screen right is +X."""
    camera = _camera()
    camera.look_at(position=[0, 0, 10], target=[0, 0, 0])
    camera.update()
    assert np.allclose(camera.get_right_vector(), [1, 0, 0], atol=1e-5), (
        f"expected +X, got {camera.get_right_vector()}")


def _press(camera, key):
    """runs one move() with a single key held, returning the displacement."""
    import glfw
    before = np.array(camera.get_position(), dtype=float)
    camera.check_pressed = lambda window, pressed, key=key: pressed == key
    camera.move(None, 1.0)
    camera.update()
    return np.array(camera.get_position(), dtype=float) - before


def test_wasd_move_in_the_directions_the_screen_shows():
    """behavioural, not structural: W must move toward what the camera looks
    at, and A/D must strafe the way the image does.

    This is the test that was missing. Fixing the right-vector sign silently
    inverted A/D, because move() consumes the right vector implicitly via
    translate(x, 0, 0) along local X -- so grepping for get_right_vector()
    callers did not reveal it."""
    import glfw
    for yaw, pitch in [(-90, 0), (0, 0), (37, 15), (200, -25)]:
        camera = _camera()
        camera.yaw, camera.pitch = yaw, pitch
        camera.set_position([0.0, 0.0, 0.0])
        camera.check_pressed = lambda window, key: False
        camera.move(None, 0.0)          # establish the basis
        camera.update()
        right, up, forward = basis_from_view(camera.matrix_view)

        moved = _press(camera, glfw.KEY_D)
        assert float(np.dot(moved, right)) > 0, (
            f"D should strafe toward screen right at yaw={yaw} pitch={pitch}, "
            f"moved {moved} against right {right}")

        camera.set_position([0.0, 0.0, 0.0])
        moved = _press(camera, glfw.KEY_A)
        assert float(np.dot(moved, right)) < 0, (
            f"A should strafe toward screen left at yaw={yaw} pitch={pitch}")

        camera.set_position([0.0, 0.0, 0.0])
        moved = _press(camera, glfw.KEY_W)
        assert float(np.dot(moved, forward)) > 0, (
            f"W should move forward at yaw={yaw} pitch={pitch}")

        camera.set_position([0.0, 0.0, 0.0])
        moved = _press(camera, glfw.KEY_S)
        assert float(np.dot(moved, forward)) < 0, (
            f"S should move backward at yaw={yaw} pitch={pitch}")


def test_space_and_shift_move_along_world_up():
    """these translate in world space, so they are independent of the camera
    basis and must not have been affected by the sign fix."""
    import glfw
    camera = _camera()
    camera.yaw, camera.pitch = 37, 15
    camera.set_position([0.0, 0.0, 0.0])
    camera.check_pressed = lambda window, key: False
    camera.move(None, 0.0)
    assert _press(camera, glfw.KEY_SPACE)[1] > 0, "SPACE should move up"
    camera.set_position([0.0, 0.0, 0.0])
    assert _press(camera, glfw.KEY_LEFT_SHIFT)[1] < 0, "SHIFT should move down"


def test_key_just_pressed_fires_once_per_press():
    """input here is polled, not event-driven, so a held key reads as
    pressed on every frame. Anything that should happen once per press
    needs an edge latch -- without one, holding the trigger key would fire
    a burst of triggers instead of one.

    Note SPACE deliberately keeps its camera binding (see the test above);
    it was the *trigger* that moved off it, because polling the same key in
    update() and then calling camera.move() fired a sweep and flew the
    camera upward at the same time."""
    import glfw
    from NightEngine import NightBase as base_module

    # a bare instance: key_just_pressed touches only window and _key_states,
    # so no GL context or window is needed
    engine = base_module.NightBase.__new__(base_module.NightBase)
    engine.window = None
    engine._key_states = {}

    held = {}
    original = base_module.glfw.get_key
    base_module.glfw.get_key = lambda window, key: (
        glfw.PRESS if held.get(key) else glfw.RELEASE)
    try:
        assert not engine.key_just_pressed(glfw.KEY_T), "fired while up"

        held[glfw.KEY_T] = True
        assert engine.key_just_pressed(glfw.KEY_T), "did not fire on press"
        for frame in range(5):
            assert not engine.key_just_pressed(glfw.KEY_T), (
                f"fired again on frame {frame} while still held")

        held[glfw.KEY_T] = False
        assert not engine.key_just_pressed(glfw.KEY_T), "fired on release"
        held[glfw.KEY_T] = True
        assert engine.key_just_pressed(glfw.KEY_T), "did not re-arm"

        # each key is latched independently
        held[glfw.KEY_E] = True
        assert engine.key_just_pressed(glfw.KEY_E), "keys share one latch"
        assert not engine.key_just_pressed(glfw.KEY_T), "T re-fired"
    finally:
        base_module.glfw.get_key = original


def test_true_up_is_unchanged_by_the_sign_convention():
    """both cross-product orderings give the same true_up, which is why
    fixing the right vector left every rendered image untouched."""
    rng = np.random.default_rng(7)
    for _ in range(50):
        forward = rng.normal(size=3)
        forward /= np.linalg.norm(forward)
        up = np.array([0.0, 1.0, 0.0])
        if abs(forward[1]) > 0.99:
            up = np.array([0.0, 0.0, 1.0])
        old_right = np.cross(up, forward)
        old_up = np.cross(forward, old_right / np.linalg.norm(old_right))
        new_right = np.cross(forward, up)
        new_up = np.cross(new_right / np.linalg.norm(new_right), forward)
        assert np.allclose(old_up / np.linalg.norm(old_up),
                           new_up / np.linalg.norm(new_up), atol=1e-6)


if __name__ == "__main__":
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith("test_") and callable(obj)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            failed += 1
            print(f"  FAIL  {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
