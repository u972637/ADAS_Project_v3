import unittest
from types import SimpleNamespace
import numpy as np

from ultralytics.engine.results import Results
from animal.inference import AnimalIdentityTracker, infer_detections


def result(rows, keypoints=None):
    return Results(
        np.zeros((300, 300, 3), np.uint8),
        path="synthetic",
        names={0: "boar", 1: "deer", 2: "torso"},
        boxes=np.array(rows, dtype=np.float32).reshape(-1, 6),
        keypoints=keypoints,
    )


class Model:
    def __init__(self, raw):
        self.raw = raw

    def predict(self, *args, **kwargs):
        return [self.raw]


class InferenceTests(unittest.TestCase):
    def test_torso_cannot_take_animal_id(self):
        tracker = AnimalIdentityTracker([0, 1])
        _, animals, _ = tracker.update(result([[50, 50, 150, 150, 0.9, 0]]))
        original_id = animals[0].track_id
        _, animals, raw = tracker.update(result([[55, 55, 145, 145, 0.9, 2]]))
        self.assertEqual(animals, ())
        self.assertIsNone(raw[0].track_id)
        _, animals, _ = tracker.update(result([[52, 50, 152, 150, 0.9, 0]]))
        self.assertEqual(animals[0].track_id, original_id)

    def test_species_change_cannot_take_other_species_id(self):
        tracker = AnimalIdentityTracker([0, 1])
        _, animals, _ = tracker.update(result([[50, 50, 150, 150, 0.9, 0]]))
        old_id = animals[0].track_id
        deer = result([[50, 50, 150, 150, 0.9, 1]])
        tracker.update(deer)
        _, animals, _ = tracker.update(deer)
        self.assertEqual(animals[0].class_id, 1)
        self.assertNotEqual(animals[0].track_id, old_id)

    def test_low_confidence_torso_survives_pipeline(self):
        service = SimpleNamespace(
            class_map={0: "boar", 1: "water_deer"},
            tracker_config="bytetrack.yaml",
            identity_tracker=None,
            min_selection_confidence=0.5,
            torso_confidence=0.2,
            torso_class_id=2,
        )
        _, ds = infer_detections(
            Model(result([[50, 50, 150, 150, 0.9, 0], [60, 60, 130, 130, 0.22, 2]])),
            service,
            None,
        )
        torso = next(d for d in ds if d.class_id == 2)
        self.assertAlmostEqual(torso.confidence, 0.22)
        self.assertIsNone(torso.track_id)
        self.assertIsNotNone(next(d for d in ds if d.class_id == 0).track_id)

    def test_empty_frames_advance_all_trackers(self):
        tracker = AnimalIdentityTracker([0, 1])
        tracker.update(result([[50, 50, 150, 150, 0.9, 0]]))
        for _ in range(3):
            _, animals, _ = tracker.update(result([]))
            self.assertEqual(animals, ())
        self.assertEqual([t.frame_id for t in tracker.trackers.values()], [4, 4])

    def test_person_proxy_uses_raw_keypoint_alignment(self):
        from tools.person_proxy import infer_person_detections

        points = np.zeros((2, 17, 3), dtype=np.float32)
        for i, cx in enumerate((80, 220)):
            for k, (dx, dy) in zip(
                (5, 6, 11, 12), ((-20, 0), (20, 0), (-20, 50), (20, 50))
            ):
                points[i, k] = [cx + dx, 80 + dy, 0.9]
        raw = result([[40, 40, 120, 200, 0.9, 0], [180, 40, 260, 200, 0.9, 0]], points)
        service = SimpleNamespace(
            class_map={0: "boar"},
            tracker_config="bytetrack.yaml",
            identity_tracker=None,
            min_selection_confidence=0.5,
            torso_confidence=0.5,
            torso_class_id=-1,
        )
        _, ds = infer_person_detections(Model(raw), service, None)
        self.assertEqual(
            [d.center for d in ds if d.class_id == -1], [(80, 105), (220, 105)]
        )
        self.assertEqual(len({d.track_id for d in ds if d.class_id == 0}), 2)
