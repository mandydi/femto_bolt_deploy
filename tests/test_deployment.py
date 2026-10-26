"""Deployment contract checks: missing geometry, malformed assets and configuration."""
import importlib.util
import json
import os
import pathlib
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('deployment_config', ROOT/'runtime/deployment_config.py')
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)

class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = pathlib.Path(self.temp.name)/'calibration.json'
        self.old = os.environ.get('CALIBRATION_PATH')
        os.environ['CALIBRATION_PATH'] = str(self.path)
        self.data = json.loads((ROOT/'config/calibration.example.json').read_text())
        config.load_calibration.cache_clear()
    def tearDown(self):
        config.load_calibration.cache_clear()
        if self.old is None: os.environ.pop('CALIBRATION_PATH', None)
        else: os.environ['CALIBRATION_PATH'] = self.old
        self.temp.cleanup()
    def load(self):
        self.path.write_text(json.dumps(self.data))
        return config.load_calibration()
    def valid(self):
        self.data.update(validated_for_deployment=True,world_frame='test_fixture')
    def test_reference_cannot_start_a_new_workcell(self):
        with self.assertRaisesRegex(ValueError,'Supply this device'): self.load()
    def test_verified_config_round_trip(self):
        self.valid()
        self.assertEqual(self.load()['camera_matrix'][0][0],2249.6287)
    def test_bad_transform_is_rejected(self):
        self.valid(); self.data['camera_to_world_mm'][3]=[1,0,0,1]
        with self.assertRaisesRegex(ValueError,'homogeneous'): self.load()
    def test_wrong_resolution_is_rejected(self):
        self.valid(); self.data['image_size']=[1280,720]
        with self.assertRaisesRegex(ValueError,'3840'): self.load()
    def test_nonfinite_intrinsic_is_rejected(self):
        self.valid(); self.data['camera_matrix'][0][0]=float('nan')
        with self.assertRaisesRegex(ValueError,'finite'): self.load()

if __name__=='__main__': unittest.main()
