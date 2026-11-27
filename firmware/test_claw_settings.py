"""Claw preset validation, safe persistence and native Tk test controls."""
import copy
import json
import tempfile
from pathlib import Path
import tkinter as tk
import unittest
from unittest.mock import patch
import miniarm_controlC as gui


def test_folder():
    # Keep disposable test data beside this test, inside the selected workspace.
    return tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent,prefix='claw_test_')


class ClawDataTests(unittest.TestCase):
    def setUp(self): self.cal=gui.ik.demo_calibration()

    def test_changes_only_two_presets_and_does_not_mutate_input(self):
        original=copy.deepcopy(self.cal.data)
        out=gui.claw_calibration(self.cal,85,45)  # either servo direction is valid
        self.assertEqual((out.claw_open,out.claw_closed),(85,45))
        self.assertEqual(out.data['joints'],original['joints'])
        self.assertEqual(out.limits(gui.CLAW),self.cal.limits(gui.CLAW))
        self.assertEqual(self.cal.data,original)

    def test_invalid_values(self):
        for opened,closed in [('',80),('nan',80),(40,'inf'),(40,70.5),(39,80),(40,91),(50,50)]:
            with self.subTest(values=(opened,closed)),self.assertRaises(ValueError):
                gui.claw_calibration(self.cal,opened,closed)

    def test_no_calibration(self):
        with self.assertRaises(ValueError): gui.claw_calibration(None,40,80)

    def test_save_preserves_metadata_and_backs_up_exact_bytes(self):
        with test_folder() as folder:
            path=Path(folder)/'calibration.json'
            data=copy.deepcopy(self.cal.data); data['operator_note']='Keep this metadata'
            original=json.dumps(data,indent=4).encode('utf-8'); path.write_bytes(original)
            out=gui.save_claw_calibration(self.cal,45,85,path)
            saved=gui.mc.Calibration.load(path)
            self.assertEqual((saved.claw_open,saved.claw_closed),(45,85))
            self.assertEqual(saved.data['joints'],data['joints'])
            self.assertEqual(saved.data['operator_note'],data['operator_note'])
            self.assertEqual(Path(str(path)+'.before_claw_edit').read_bytes(),original)
            self.assertEqual(out.data,saved.data)

    def test_save_preserves_legacy_angle_convention(self):
        data=copy.deepcopy(self.cal.data); del data['angle_convention']
        cal=gui.mc.Calibration(data)
        with test_folder() as folder:
            path=Path(folder)/'calibration.json'; gui.mc.Calibration.save(data,path)
            out=gui.save_claw_calibration(cal,45,85,path)
            self.assertEqual(out.angle_convention,'legacy')
            self.assertNotIn('angle_convention',out.data)
            self.assertEqual(out.data['joints'],data['joints'])

    def test_external_joint_edit_is_not_overwritten(self):
        data=copy.deepcopy(self.cal.data); data['joints']['base']['points'][1][1]=95
        with test_folder() as folder:
            path=Path(folder)/'calibration.json'; gui.mc.Calibration.save(data,path)
            original=path.read_bytes()
            with self.assertRaisesRegex(ValueError,'changed on disk'):
                gui.save_claw_calibration(self.cal,45,85,path)
            self.assertEqual(path.read_bytes(),original)

    def test_failed_replace_keeps_file_and_cleans_temporary(self):
        with test_folder() as folder:
            path=Path(folder)/'calibration.json'; gui.mc.Calibration.save(self.cal.data,path)
            original=path.read_bytes()
            with patch.object(gui.os,'replace',side_effect=PermissionError('test')),self.assertRaises(PermissionError):
                gui.save_claw_calibration(self.cal,45,85,path)
            self.assertEqual(path.read_bytes(),original)
            self.assertFalse(list(Path(folder).glob('.claw_*.json')))


class ClawGuiTests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk(); self.root.withdraw()
        self.app=gui.ArmApp(self.root,simulate=True)
        self.sent=[]; original=self.app.link.send
        self.app.link.send=lambda line:(self.sent.append(line),original(line))[-1]

    def tearDown(self): self.app.close()

    def dialog(self):
        original=tk.Toplevel
        def hidden(*args,**kwargs):
            window=original(*args,**kwargs); window.withdraw(); return window
        with patch.object(gui.tk,'Toplevel',side_effect=hidden): self.app.claw_settings()
        return self.app._claw_controls

    def test_apply_does_not_move_and_invalidates_plans(self):
        self.app._pp_preview=object()
        self.app.apply_claw_settings(45,85)
        self.assertEqual(self.sent,[])
        self.assertEqual(self.app.positions[0],40)
        self.assertEqual((self.app.cal.claw_open,self.app.cal.claw_closed),(45,85))
        self.assertIsNone(self.app._pp_preview)
        self.assertEqual(self.app.start_pose()[0],45)

    def test_busy_rejects_preset_changes(self):
        self.app.busy=True
        with self.assertRaises(ValueError): self.app.apply_claw_settings(45,85)
        self.assertEqual(self.app.cal.claw_open,40)

    def test_synthetic_save_cannot_touch_hardware_file(self):
        with patch.object(gui,'save_claw_calibration') as saver,self.assertRaises(ValueError):
            self.app.apply_claw_settings(45,85,save=True)
        saver.assert_not_called()

    def test_test_closed_only_changes_s1(self):
        controls=self.dialog(); before=list(self.app.positions)
        controls['values'][0].set('45'); controls['values'][1].set('85')
        controls['test'][1].invoke()
        packets=[line for line in self.sent if line.startswith('T ')]
        self.assertEqual(len(packets),1)
        values=[float(v) for v in packets[0].split()[2:]]
        self.assertEqual(values[0],85)
        self.assertEqual(values[1:],before[1:])
        self.assertTrue(self.app.busy)
        self.assertEqual(str(controls['test'][0].cget('state')),'disabled')

    def test_test_open_uses_edited_preset(self):
        controls=self.dialog()
        controls['values'][0].set('45'); controls['values'][1].set('85')
        controls['test'][0].invoke()
        self.assertEqual(float(self.sent[0].split()[2]),45)

    def test_invalid_dialog_value_sends_nothing(self):
        controls=self.dialog(); controls['values'][0].set('39')
        controls['test'][0].invoke()
        self.assertEqual(self.sent,[])
        self.assertEqual(self.app.cal.claw_open,40)
        self.assertIn('range',controls['message'].get())

    def test_knob_mode_disables_tests(self):
        self.app.set_mode('K'); self.app.poll()
        controls=self.dialog()
        self.assertTrue(all(str(b.cget('state'))=='disabled' for b in controls['test']))

    def test_save_from_gui_preserves_initialization_and_moves_nothing(self):
        self.app.synthetic=False
        with test_folder() as folder:
            path=Path(folder)/'calibration.json'; gui.mc.Calibration.save(self.app.cal.data,path)
            controls=self.dialog(); controls['values'][0].set('45'); controls['values'][1].set('85')
            with patch.object(gui,'calibration_path',return_value=str(path)): controls['save'].invoke()
            self.assertEqual(gui.mc.Calibration.load(path).claw_closed,85)
        self.assertTrue(self.app.initialized)
        self.assertEqual(self.sent,[])

    def test_new_values_reach_pick_place_planner(self):
        self.app.apply_claw_settings(45,85)
        plan=gui.pp.build_plan(self.app.m,self.app.cal,self.app.positions,[0,130,50],self.app._pp_settings)
        close=next(s for s in plan.steps if s.label=='Close claw / grasp')
        release=next(s for s in plan.steps if s.label=='Open claw / release')
        self.assertEqual(close.pose[0],85); self.assertEqual(release.pose[0],45)


if __name__=='__main__': unittest.main()
