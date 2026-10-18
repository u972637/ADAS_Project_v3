"""Raw inference → species-isolated animal tracks + untracked torso detections."""

from dataclasses import replace
from types import SimpleNamespace


from animal.perception import Detection, read_detections


class AnimalIdentityTracker:
    def __init__(self, class_ids, tracker_config="bytetrack.yaml"):
        from ultralytics.trackers.byte_tracker import BYTETracker
        from ultralytics.utils import YAML
        from ultralytics.utils.checks import check_yaml

        config = YAML.load(check_yaml(tracker_config))
        if config.get("tracker_type") != "bytetrack":
            raise ValueError("Species isolation currently supports ByteTrack only")
        self.trackers = {
            cid: BYTETracker(SimpleNamespace(**config)) for cid in sorted(class_ids)
        }
        self._ids = {}
        self._next_id = 1

    def update(self, result):
        size, raw = read_detections(result)
        boxes = result.boxes.cpu().numpy()
        tracked = []
        # All trackers advance on every frame, including when their species is absent.
        for class_id, tracker in self.trackers.items():
            subset = boxes[boxes.cls == class_id]
            for row in tracker.update(subset):
                key = (class_id, int(row[4]))
                if key not in self._ids:
                    self._ids[key] = self._next_id
                    self._next_id += 1
                tracked.append(
                    Detection(
                        class_id,
                        float(row[5]),
                        tuple(float(v) for v in row[:4]),
                        self._ids[key],
                    )
                )
        # Torsos never consume an animal ID or ByteTrack's higher new-track threshold.
        return size, tuple(tracked), raw


def infer_detections(model, service, frame, *, person_proxy=False):
    conf = min(service.min_selection_confidence, service.torso_confidence)
    results = model.predict(frame, conf=conf, verbose=False)
    if len(results) != 1:
        raise ValueError("model must return one result per frame")
    result = results[0]
    if service.identity_tracker is None:
        service.identity_tracker = AnimalIdentityTracker(
            service.class_map, service.tracker_config
        )
    size, animals, raw = service.identity_tracker.update(result)
    if person_proxy:
        from animal.person_proxy import read_person_detections

        _, pose_detections = read_person_detections(result)
        torsos = [d for d in pose_detections if d.class_id == service.torso_class_id]
    else:
        torsos = [
            replace(d, track_id=None)
            for d in raw
            if d.class_id == service.torso_class_id
        ]
    return size, (*animals, *torsos)
