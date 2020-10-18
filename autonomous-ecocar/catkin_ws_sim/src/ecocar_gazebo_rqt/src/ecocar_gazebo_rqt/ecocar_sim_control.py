import os
import rospy
import rospkg

import subprocess # for calling directly instead of using the ros api
import signal

import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsScene
from python_qt_binding.QtCore import QTimer, QThread, Qt, QPoint
from python_qt_binding.QtGui import QPainter

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

from ecocar_gazebo_msgs.msg import SteeringData, MotorData, Track, DragData
from dynamo_msgs.msg import BrakeStepper

from std_srvs.srv import Empty

class EcocarSimControl(Plugin):

    def __init__(self, context):
        super(EcocarSimControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('EcocarSimControl')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('ecocar_gazebo_rqt'), 'resource', 'EcocarSimControl.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')

        self._widget._start.clicked.connect(self.start_gazebo)
        self._widget._stop.clicked.connect(self.stop_gazebo)
        self._widget._reset_data.clicked.connect(self.reset_data)
        self._widget._reset_simulation.clicked.connect(self.reset_simulation)
        self._widget._challenge_selector.setCurrentIndex(2)

        scene = QGraphicsScene()
        self._widget.graphicsView.setScene(scene)
        self._widget.graphicsView.setStyleSheet('background-color: white')

        self._widget._circle = Circle(self._widget, 240, 160, 10)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.started = False # simulation started bool

        self.launch = None
        self._t = None
        self._stopping = False
        self.last_pose_time = time.time()

        # Stuff for debug track drawing
        self.drawn_track = False
        self.map_scale = 1
        self.car_marker = None

        # Timer for UI updates
        self._timer = QTimer()
        self._timer.timeout.connect(self.update)
        self._timer.start(100)

        # Subscribe to ROS topics
        rospy.Subscriber("sim/crashed", Bool, self.crashed_callback)
        rospy.Subscriber("sim/debug/pose", Pose, self.debug_pose_callback)
        rospy.Subscriber("sim/debug/steering", SteeringData, self.debug_steering_callback)
        rospy.Subscriber("sim/debug/motor", MotorData, self.debug_motor_callback)

        rospy.Subscriber("sim/debug/drag", DragData, self.debug_drag_callback)

        rospy.Subscriber("cmd_brake_power", BrakeStepper, self.brake_stepper_callback) # use a debug output instead?

        rospy.Subscriber("sim/track", Track, self.track_callback)

        self.reset_simulation_service = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self._resetting = False

        self.sim_state = SimulationState()

    def shutdown_plugin(self):
        self._timer.stop()
        # If user closes rqt before stopping, make sure to stop it
        # Otherwise you have to stop the process manually..
        self.stop_gazebo()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def update(self):
        # Update main GUI elements
        # Check time since last pose msg
        if self.started and not self._resetting and time.time()-self.last_pose_time > 1.0:
            # TODO better way of detecting simulation stopped than looking at debug pose msg
            self._widget._label.setText("Stopped")
            self._stopping = False
            self.launch = None
            self.started = False
            self.car_marker = None

        self._widget._stop.setDisabled((not self.started))
        self._widget._start.setDisabled(self.started)
        self._widget._reset_simulation.setDisabled((not self.started))
        
        self._widget._challenge_selector.setDisabled(self.started)

        if self.started:
            # Data visualization
            # Has ecocar crashed?
            if self.sim_state.ecocar_crashed:
                self._widget._circle.setColor(Qt.red)
            else:
                self._widget._circle.setColor(Qt.green)

            # Track
            if not self.drawn_track and len(self.sim_state.track.points) > 0:
                self._widget.graphicsView.scene().clear()
                w = 451.0
                h = 180.0

                max_x = max(self.sim_state.track.points, key=lambda p: p.x).x
                min_x = min(self.sim_state.track.points, key=lambda p: p.x).x

                max_y = max(self.sim_state.track.points, key=lambda p: p.y).y
                min_y = min(self.sim_state.track.points, key=lambda p: p.y).y

                print(max_x, min_x, max_y, min_y)

                try:
                    self.map_scale = min((w-7)/(max_x-min_x), (h-7)/(max_y - min_y))
                except ZeroDivisionError:
                    # Could happen if max_x = min_x or max_y = min_y
                    self.map_scale = 1.0

                for b in self.sim_state.track.points:
                    self._widget.graphicsView.scene().addEllipse(self.map_scale*b.x, self.map_scale*b.y, 5, 5)
                self.drawn_track = True

            # Pose
            if self.drawn_track:
                if self.car_marker is not None:
                    self.car_marker.setPos(self.map_scale*self.sim_state.pose.position.x, self.map_scale*self.sim_state.pose.position.y)
                else:
                    self.car_marker = self._widget.graphicsView.scene().addEllipse(self.map_scale*self.sim_state.pose.position.x, 
                                                                                self.map_scale*self.sim_state.pose.position.y,
                                                                                5, 
                                                                                5, 
                                                                                pen=Qt.red,
                                                                                brush=Qt.red)
            # Steering debug
            self._widget._steering_angle_cmd.setText("{:.2f} deg".format(self.sim_state.steering.command_angle))
            self._widget._steering_angle_left.setText("{:.2f} deg".format(self.sim_state.steering.current_angle_left))
            self._widget._steering_angle_right.setText("{:.2f} deg".format(self.sim_state.steering.current_angle_right))

            # Brake
            self._widget._brake_pressure.setText("{:.2f} bar".format(self.sim_state.brake.brake_power))
            if self.sim_state.brake.brake_stepper_engaged:
                self._widget._brake_pressure.setStyleSheet('color: green')
            else:
                self._widget._brake_pressure.setStyleSheet('color: red')

            # Motor debug
            self._widget._motor_omega.setText("{:.2f} rad/s".format(self.sim_state.motor.omega_wheel))
            self._widget._motor_torque.setText("{:.2f} Nm".format(self.sim_state.motor.wheel_torque))

            # Drag force
            self._widget._drag_force.setText("{:.2f} N".format(self.sim_state.drag.drag_force))

    def crashed_callback(self, msg):
        self.sim_state.ecocar_crashed = msg.data

    def debug_pose_callback(self, pose):
        self.last_pose_time = time.time()

        if not self._stopping and not self._resetting and not self.started:
            print "Started Gazebo"
            self._widget._label.setText("Started")
            self.started = True
        elif self._resetting:
            # not resetting any more
            self._resetting = False

        self.sim_state.pose = pose

    def debug_steering_callback(self, msg):
        self.sim_state.steering = msg

    def debug_motor_callback(self, msg):
        self.sim_state.motor = msg

    def debug_drag_callback(self, msg):
        self.sim_state.drag = msg

    def track_callback(self, msg):
        self.sim_state.track = msg

    def brake_stepper_callback(self, msg):
        self.sim_state.brake = msg

    def reset_data(self):
        self.sim_state = SimulationState()    
        self._widget.graphicsView.scene().clear() 
        self.drawn_track = False
        self.car_marker = None

    def reset_simulation(self):
        if self.started:
            self.reset_simulation_service()
            self._resetting = True
        else:
            print("Can't reset simulation when it's not started")

    def start_gazebo(self):
        # Check whether it's started already
        if self.launch:
            print "Already started"
            return

        print "Attempting to start Gazebo"
        self._widget._label.setText("Starting..")

        self.reset_data()

        cmd = 'roslaunch ecocar_gazebo ' \
              '{}.launch ' \
              'gui:={}'.format(self._widget._challenge_selector.currentText(),
                                self._widget._gui_checkbox.isChecked())

        print(cmd)
        self.launch = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                       shell=True, preexec_fn=os.setsid) 

    def stop_gazebo(self):
        # check if it is even running first..
        if self.launch:
            if not self._stopping:
                self._widget._label.setText("Stopping..")
                self._stopping = True
                # Kill cmd
                os.killpg(os.getpgid(self.launch.pid), signal.SIGTERM)
            else:
                print "Already trying to stop"
        else:
            print "Not launched.."

class SimulationState:
    def __init__(self):
        self.ecocar_crashed = False
        self.percentage_completed = 0
        self.pose = Pose()
        self.steering = SteeringData()
        self.brake = BrakeStepper()
        self.motor = MotorData()
        self.drag = DragData()
        self.track = Track()

class Circle(QWidget):
    def __init__(self, parent=None, x_pos=0, y_pos=0, r=50):
        QWidget.__init__(self, parent)
        self.r = r
        self.setGeometry(x_pos, y_pos, 2*r, 2*r)
        self.c = Qt.red

    def setColor(self, color):
        if color != self.c:
            self.c = color
            self.repaint()

    def paintEvent(self, event):
        paint = QPainter()
        paint.begin(self)
        # optional
        paint.setRenderHint(QPainter.Antialiasing)
        #paint.setRenderHint(QPainter.HighQualityAntialiasing)
        # for circle make the ellipse radii match
        radx = self.r
        rady = self.r
        # draw red circles
        paint.setBrush(self.c)
        paint.setPen(self.c)
        paint.drawEllipse(QPoint(self.r,self.r), radx, rady)
        paint.end()