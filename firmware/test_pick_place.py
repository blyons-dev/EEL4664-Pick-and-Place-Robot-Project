"""Focused planner and native Tk/emulator regressions; never opens a COM port."""
import queue
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch
import numpy as np
import miniarm_controlC as gui
import miniarm_pick_place as pp


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cal=gui.ik.demo_calibration(); cls.m=gui.ik.model(gui.GEOMETRY,cls.cal)
        cls.start=gui.ik.q_to_servo(cls.cal,np.deg2rad([90,90,0,0]),cls.cal.claw_open)
        cls.settings=pp.CupSettings()
        cls.plan=pp.build_plan(cls.m,cls.cal,cls.start,[0,130,50],cls.settings)

    def test_dimensions_and_release(self):
        s=self.settings
        self.assertEqual((s.x,s.y,s.rim_z,s.diameter),(127,0,127,101.6))
        self.assertGreater(s.release_xyz[2]-s.object_radius,s.rim_z)

    def test_order_and_claw(self):
        labels=[s.label for s in self.plan.steps]
        self.assertEqual(len(labels),9)
        close=labels.index('Close claw / grasp'); release=labels.index('Open claw / release')
        self.assertLess(close,release)
        for step in self.plan.steps[close:release]: self.assertEqual(step.pose[0],self.cal.claw_closed)
        for step in self.plan.steps[release:]: self.assertEqual(step.pose[0],self.cal.claw_open)
        self.assertTrue(labels[-1].startswith('Retreat'))

    def test_every_leg_is_prevalidated(self):
        start=self.start
        for step in self.plan.steps:
            gui.ik.validate_coordinated_leg(self.m,self.cal,start,step.pose)
            pp.validate_cup_leg(self.m,self.cal,start,step.pose,self.settings,step.holding)
            self.assertGreaterEqual(step.duration_ms,2000)
            self.assertLessEqual(step.duration_ms,25000)
            start=step.pose

    def test_bad_settings_rejected(self):
        for s in [pp.CupSettings(rim_z=float('nan')),pp.CupSettings(diameter=30),
                  pp.CupSettings(lift=-1),pp.CupSettings(dwell=0)]:
            with self.subTest(s=s),self.assertRaises(ValueError): s.validate()

    def test_unreachable_pick_rejected(self):
        with self.assertRaises(ValueError):
            pp.build_plan(self.m,self.cal,self.start,[1000,1000,50],self.settings)

    def test_cancelled_preflight_rejected(self):
        with self.assertRaisesRegex(ValueError,'cancelled'):
            pp.build_plan(self.m,self.cal,self.start,[0,130,50],self.settings,cancel=lambda:True)

    def test_cup_collision_rejected(self):
        horizontal=gui.ik.q_to_servo(self.cal,np.deg2rad([0,0,0,0]),self.cal.claw_open)
        with self.assertRaises(ValueError):
            pp.validate_cup_leg(self.m,self.cal,self.start,horizontal,self.settings)

    def test_box_distance(self):
        lo=np.zeros(3); hi=np.ones(3)
        self.assertEqual(pp.segment_box_distance(np.array([-2.,.5,.5]),np.array([2.,.5,.5]),lo,hi),0)
        self.assertAlmostEqual(pp.segment_box_distance(np.array([-1.,2.,.5]),np.array([2.,2.,.5]),lo,hi),1)
        self.assertAlmostEqual(pp.segment_box_distance(np.array([2.,2.,2.]),np.array([3.,3.,3.]),lo,hi),np.sqrt(3))

    def test_opposite_wrist_direction_and_measured_claw(self):
        data=dict(angle_convention='joint',joints={
            'base':dict(points=[[0,0],[90,90],[180,180]],min=0,max=180),
            'arm':dict(points=[[0,180],[90,90],[180,0]],min=0,max=180),
            'forearm':dict(points=[[0,90],[45,48],[90,0],[135,-42],[180,-90]],min=0,max=180),
            'wrist':dict(points=[[0,-90],[90,0],[180,90]],min=0,max=180)},
            claw=dict(open=70,closed=80,min=65,max=85))
        cal=gui.mc.Calibration(data); m=gui.ik.model(gui.GEOMETRY,cal)
        start=gui.ik.q_to_servo(cal,np.deg2rad([90,90,0,0]),cal.claw_open)
        plan=pp.build_plan(m,cal,start,[0,130,50],self.settings)
        self.assertEqual(plan.steps[-1].pose[0],70)
        self.assertTrue(any(step.pose[0]==80 for step in plan.steps))


class NativeGuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        PlannerTests.setUpClass()

    def setUp(self):
        self.root=tk.Tk(); self.root.withdraw()
        self.app=gui.ArmApp(self.root,simulate=True)
        self.root.update_idletasks()
        self.sent=[]; original=self.app.link.send
        self.app.link.send=lambda line:(self.sent.append(line),original(line))[-1]
        for var,value in zip(self.app.target_values,[0,130,50]): var.set(str(value))
        self.app.poll()

    def tearDown(self):
        self.app.close()

    def install_plan(self):
        plan=PlannerTests.plan
        self.app._pp_messages.put((self.app._pp_generation,'ready',plan))
        self.app.poll_pick_place()
        self.assertIsNotNone(self.app._pp_preview)
        return plan

    def drive(self,stop_at_grasp=False):
        clock=[1000.]
        with patch.object(gui.time,'monotonic',side_effect=lambda:clock[0]):
            self.app.run_pick_place()
            for _ in range(1000):
                clock[0]+=.25; self.app.poll()
                if stop_at_grasp and self.app._pp_delay is not None and self.app._pp_current_label=='Close claw / grasp':
                    return
                if self.app._pp_delay is not None:
                    self.root.after_cancel(self.app._pp_delay); self.app._pp_delay=None
                    self.app.send_pick_place_step()
                if not self.app._pp_running: return
        self.fail('Task did not finish')

    def test_move_arm_remains_separate(self):
        self.assertEqual(self.app.move_button.cget('text'),'Move Arm')
        self.assertEqual(self.app.pp_run_button.cget('text'),'Run Pick & Place')
        self.assertEqual(str(self.app.pp_run_button.cget('state')),'disabled')

    def test_preview_has_no_serial_commands(self):
        self.install_plan()
        self.assertEqual(self.sent,[])
        self.assertEqual(str(self.app.pp_run_button.cget('state')),'normal')

    def test_real_background_preflight(self):
        self.app.preview_pick_place()
        deadline=time.monotonic()+20
        while self.app._pp_planning and time.monotonic()<deadline:
            time.sleep(.01); self.app.poll()
        self.assertIsNotNone(self.app._pp_preview,self.app.feedback_message)
        self.assertEqual(self.sent,[])

    def test_complete_sequence_waits_for_acks(self):
        plan=self.install_plan(); self.drive()
        targets=[line for line in self.sent if line.startswith('T ')]
        self.assertEqual(len(targets),len(plan.steps))
        for line,step in zip(targets,plan.steps):
            np.testing.assert_allclose([float(v) for v in line.split()[2:]],step.pose)
        np.testing.assert_allclose(self.app.positions,plan.steps[-1].pose)
        self.assertFalse(self.app.busy)
        self.assertIsNone(self.app._pp_preview)

    def test_stop_during_grasp_dwell_cancels_future_sends(self):
        self.install_plan(); self.drive(stop_at_grasp=True)
        self.assertEqual(self.app.positions[0],self.app.cal.claw_closed)
        self.app.stop(); count=len([l for l in self.sent if l.startswith('T ')])
        self.app.poll(); self.app.send_pick_place_step()
        self.assertIsNone(self.app._pp_delay)
        self.assertFalse(self.app._pp_running)
        self.assertEqual(len([l for l in self.sent if l.startswith('T ')]),count)
        self.assertEqual(self.app.positions[0],self.app.cal.claw_closed)

    def test_start_change_rejects_run(self):
        self.install_plan(); self.app.positions[4]+=1
        self.app.run_pick_place()
        self.assertFalse(any(l.startswith('T ') for l in self.sent))

    def test_input_edit_invalidates_run(self):
        self.install_plan(); self.app.target_values[0].set('15')
        self.assertIsNone(self.app._pp_preview)
        self.app.run_pick_place()
        self.assertEqual(self.sent,[])

    def test_cancelled_generation_cannot_install_plan(self):
        generation=self.app._pp_generation; self.app.stop()
        self.app._pp_messages.put((generation,'ready',PlannerTests.plan))
        self.app.poll_pick_place()
        self.assertIsNone(self.app._pp_preview)

    def test_only_one_target_arm_drawn(self):
        self.install_plan(); self.app.canvas.config(width=600,height=500)
        self.root.update_idletasks(); self.app.redraw_preview()
        self.assertFalse(self.app.canvas.find_withtag('unsent_link'))

    def test_disconnect_cancels_task_during_pause(self):
        self.install_plan(); self.drive(stop_at_grasp=True)
        self.app.inbox.put('LINK_ERROR test disconnect'); self.app.poll()
        count=len([l for l in self.sent if l.startswith('T ')])
        self.app.send_pick_place_step()
        self.assertFalse(self.app.ready)
        self.assertFalse(self.app.initialized)
        self.assertIsNone(self.app._pp_delay)
        self.assertEqual(len([l for l in self.sent if l.startswith('T ')]),count)

    def test_position_report_keeps_target_static_and_invalidates_stale_task(self):
        plan=self.install_plan(); frozen=self.app._target_points.copy()
        self.app.positions[4]+=2; self.app.update_position()
        np.testing.assert_array_equal(self.app._target_points,frozen)
        self.assertIsNone(self.app._pp_preview)

    def test_preflight_failure_never_sends_motion(self):
        with patch.object(pp,'build_plan',side_effect=ValueError('release blocked')):
            self.app.preview_pick_place()
            deadline=time.monotonic()+3
            while self.app._pp_planning and time.monotonic()<deadline:
                time.sleep(.01); self.app.poll()
        self.assertIsNone(self.app._pp_preview)
        self.assertFalse(self.app.busy)
        self.assertTrue(self.app.initialized)
        self.assertEqual(self.sent,[])

    def test_original_move_does_not_close_claw(self):
        self.app.calculate(); self.assertIsNotNone(self.app.preview,self.app.feedback_message)
        claw=self.app.positions[0]
        self.app.move()
        timed=[l for l in self.sent if l.startswith('T ')]
        self.assertEqual(len(timed),1)
        self.assertEqual(float(timed[0].split()[2]),claw)
        self.assertFalse(self.app._pp_running)


if __name__=='__main__': unittest.main()
