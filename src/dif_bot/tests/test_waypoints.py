import importlib.util
import tempfile
import unittest
from pathlib import Path
import yaml
spec=importlib.util.spec_from_file_location('mission',Path(__file__).parents[1]/'scripts/mission_node.py')
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
class WaypointValidation(unittest.TestCase):
    def load(self,data):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'points.yaml'; p.write_text(yaml.safe_dump(data)); return module.load_waypoints(p)
    def test_valid(self):
        self.assertEqual(len(self.load({'frame_id':'map','waypoints':[{'x':0.,'y':-.7,'yaw':0.}]})),1)
    def test_invalid(self):
        for data in [None,{'frame_id':'odom','waypoints':[]},{'frame_id':'map','waypoints':[]},{'frame_id':'map','waypoints':[{'x':float('nan'),'y':0,'yaw':0}]},{'frame_id':'map','waypoints':[{'x':True,'y':0,'yaw':0}]},{'frame_id':'map','waypoints':[{'x':0,'y':0}]}]:
            with self.subTest(data=data), self.assertRaises(ValueError): self.load(data)
if __name__=='__main__': unittest.main()
