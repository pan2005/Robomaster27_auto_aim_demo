import pathlib
import sys
import tempfile
import unittest

import cv2
import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from calibration import IntrinsicCalibrationSession, rotate_rgb  # noqa: E402


class CalibrationTests(unittest.TestCase):
    def test_clockwise_rotation_matches_display_contract(self):
        image = np.array([
            [[1, 0, 0], [2, 0, 0], [3, 0, 0]],
            [[4, 0, 0], [5, 0, 0], [6, 0, 0]],
        ], dtype=np.uint8)
        rotated = rotate_rgb(image, 90)
        self.assertEqual(rotated.shape, (3, 2, 3))
        self.assertEqual(rotated[:, :, 0].tolist(), [[4, 1], [5, 2], [6, 3]])

    def test_supported_rotations(self):
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        self.assertEqual(rotate_rgb(image, 0).shape, (4, 6, 3))
        self.assertEqual(rotate_rgb(image, 180).shape, (4, 6, 3))
        self.assertEqual(rotate_rgb(image, 270).shape, (6, 4, 3))
        with self.assertRaises(ValueError):
            rotate_rgb(image, 45)

    def test_solver_and_yaml_with_synthetic_observations(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = IntrinsicCalibrationSession(pathlib.Path(temporary))
            session.image_size = (1080, 1440)
            object_points = np.zeros((70, 3), np.float32)
            object_points[:, :2] = (
                np.mgrid[0:10, 0:7].T.reshape(-1, 2) * 40.0)
            matrix = np.array([
                [1180.0, 0.0, 540.0],
                [0.0, 1195.0, 720.0],
                [0.0, 0.0, 1.0],
            ])
            distortion = np.zeros(5)
            for index in range(15):
                row, column = divmod(index, 5)
                rvec = np.array([
                    -0.22 + row * 0.20,
                    -0.24 + column * 0.12,
                    -0.08 + index * 0.011,
                ])
                tvec = np.array([
                    -210.0 + column * 80.0,
                    -145.0 + row * 110.0,
                    1150.0 + (index % 4) * 130.0,
                ])
                projected, _ = cv2.projectPoints(
                    object_points, rvec, tvec, matrix, distortion)
                session.image_points.append(projected.astype(np.float32))
                session.image_paths.append(pathlib.Path(f"synthetic_{index}.png"))

            result = session.calibrate()
            self.assertLess(result.mean_reprojection_error, 0.05)
            self.assertTrue(result.output_path.is_file())
            with result.output_path.open(encoding="utf-8") as stream:
                output = yaml.safe_load(stream)
            self.assertEqual(output["image_width"], 1080)
            self.assertEqual(output["image_height"], 1440)
            self.assertEqual(output["rotation_degrees_clockwise"], 90)
            self.assertEqual(len(output["camera_matrix"]), 9)
            self.assertEqual(len(output["distort_coeffs"]), 5)

    def test_saved_session_can_be_restored(self):
        with tempfile.TemporaryDirectory() as temporary:
            image = np.full((500, 700, 3), 255, dtype=np.uint8)
            for row in range(7):
                for column in range(10):
                    cv2.circle(image, (100 + column * 50, 100 + row * 50),
                               12, (0, 0, 0), -1)
            session = IntrinsicCalibrationSession(
                pathlib.Path(temporary), 10, 7, 20.0, 0)
            self.assertTrue(session.capture(image).success)

            restored, skipped = IntrinsicCalibrationSession.load(session.folder)
            self.assertEqual(restored.count, 1)
            self.assertEqual(skipped, 0)
            self.assertEqual(restored.spacing_mm, 20.0)
            self.assertEqual(restored.rotation_degrees, 0)
            self.assertEqual(restored.image_size, (700, 500))


if __name__ == "__main__":
    unittest.main()
