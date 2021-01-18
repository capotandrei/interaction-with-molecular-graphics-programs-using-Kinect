from math import *
import ctypes
import sys
import time
import math

from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime
import pygame

# colors for drawing different bodies
SKELETON_COLORS = [pygame.color.THECOLORS["red"],
                   pygame.color.THECOLORS["blue"],
                   pygame.color.THECOLORS["green"],
                   pygame.color.THECOLORS["orange"],
                   pygame.color.THECOLORS["purple"],
                   pygame.color.THECOLORS["yellow"],
                   pygame.color.THECOLORS["violet"]]

start = time.time()


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1),
                                               pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect
        # color frame size
        self._frame_surface = pygame.Surface(
            (self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data
        self._bodies = None

        # here we will store joints data
        self.joints = None
        self.tblJoins = []

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState
        joint1State = joints[joint1].TrackingState

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except ValueError:  # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_SpineMid)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft)

        # Right Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight,
                            PyKinectV2.JointType_ElbowRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight,
                            PyKinectV2.JointType_WristRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_HandRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight,
                            PyKinectV2.JointType_HandTipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_ThumbRight)

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft,
                            PyKinectV2.JointType_ElbowLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft,
                            PyKinectV2.JointType_HandTipLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft)

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight,
                            PyKinectV2.JointType_AnkleRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight,
                            PyKinectV2.JointType_FootRight)

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft)

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def run(self):
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            # --- Game logic should go here

            # --- Getting frames and drawing
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None:
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked:
                        continue

                    self.joints = body.joints
                    # Head angles recognition
                    self.getTblJoints()
                    self.head_tracking()

                    # convert joint coordinates to color space
                    joint_points = self._kinect.body_joints_to_color_space(self.joints)
                    self.draw_body(self.joints, joint_points, SKELETON_COLORS[i])

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height))
            self._screen.blit(surface_to_draw, (0, 0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()

    @staticmethod
    def write_to_file(data_input):
        try:
            with open("head_position.csv", "a") as textfile:
                textfile.write(str(data_input) + "\n")
            textfile.close()
        except ValueError:
            return

    def getTblJoints(self):
        self.tblJoints = []
        for i in range(0, 25):
            tup = (self.joints[i].Position.x, self.joints[i].Position.z, self.joints[i].Position.y)
            if i == 3:
                end = time.time()
                if int(end - start) % 2 == 0:
                    print(tup)
                self.write_to_file(str(self.joints[i].Position.x) + ',' + str(self.joints[i].Position.y) + ','
                                   + str(self.joints[i].Position.z))
            self.tblJoints.append(tup)

    # Executes the 4 transformations
    def schimbareSist(self, joint, shift, unghiuri):
        joint = self.translatie(joint, shift)
        joint = self.rotatieZ(joint, unghiuri[2])
        joint = self.rotatieY(joint, unghiuri[1])
        joint = self.rotatieX(joint, unghiuri[0])
        return joint

    # Translation of a joint with shiftOr
    @staticmethod
    def translatie(joint, shiftOr):
        jointNew = (joint[0] - shiftOr[0], joint[1] - shiftOr[1], joint[2] - shiftOr[2])
        return jointNew

    # Roll the joint in YOZ plan (around OX) with angle
    @staticmethod
    def rotatieX(joint, angle):
        x = joint[0]
        y = math.cos(angle) * joint[1] + math.sin(angle) * joint[2]
        z = -math.sin(angle) * joint[1] + math.cos(angle) * joint[2]
        jointNew = (x, y, z)
        return jointNew

    # Roll the joint in XOZ plan (around OY) with angle
    @staticmethod
    def rotatieY(joint, angle):
        x = math.cos(angle) * joint[0] - math.sin(angle) * joint[2]
        y = joint[1]
        z = math.sin(angle) * joint[0] + math.cos(angle) * joint[2]
        jointNew = (x, y, z)
        return jointNew

    # Roll the joint in XOY plan (around OZ) with angle
    @staticmethod
    def rotatieZ(joint, angle):
        x = math.cos(angle) * joint[0] + math.sin(angle) * joint[1]
        y = -math.sin(angle) * joint[0] + math.cos(angle) * joint[1]
        z = joint[2]
        jointNew = (x, y, z)
        return jointNew

    def setCartezian(self, referinta):
        P1 = self.tblJoints[referinta[0]]
        P2 = self.tblJoints[referinta[1]]
        P3 = self.tblJoints[referinta[2]]
        shift = P1

        # P1 becomes origin
        P1 = self.translatie(P1, shift)
        P2 = self.translatie(P2, shift)
        P3 = self.translatie(P3, shift)

        unghiZ = math.atan(P2[1] / P2[0])
        P1 = self.rotatieZ(P1, unghiZ)
        P2 = self.rotatieZ(P2, unghiZ)
        P3 = self.rotatieZ(P3, unghiZ)

        unghiY = -math.atan(P2[2] / P2[0])  # z / x
        P1 = self.rotatieY(P1, unghiY)
        P2 = self.rotatieY(P2, unghiY)
        P3 = self.rotatieY(P3, unghiY)

        unghiX = -math.atan(P3[1] / P3[2])  # y / z
        P1 = self.rotatieX(P1, unghiX)
        P2 = self.rotatieX(P2, unghiX)
        P3 = self.rotatieX(P3, unghiX)

        shift = (unghiX, unghiY, unghiZ, shift[0], shift[1], shift[2])
        return shift

    @staticmethod
    def getAngle(Joint1, Joint2, Joint3):

        a = sqrt((Joint1[0] - Joint3[0]) ** 2 + (Joint1[1] - Joint3[1]) ** 2 +
                 (Joint1[2] - Joint3[2]) ** 2)
        b = sqrt((Joint1[0] - Joint2[0]) ** 2 + (Joint1[1] - Joint2[1]) ** 2 +
                 (Joint1[2] - Joint2[2]) ** 2)
        c = sqrt((Joint2[0] - Joint3[0]) ** 2 + (Joint2[1] - Joint3[1]) ** 2 +
                 (Joint2[2] - Joint3[2]) ** 2)

        if b == 0:
            b = 0.0001
        if c == 0:
            c = 0.0001
        Angle = acos((-a ** 2 + b ** 2 + c ** 2) / (2 * b * c))
        return Angle

    # Creates joints
    def getJoints(self, shiftOrigine, unghiuri):
        Neck = self.schimbareSist(self.tblJoints[2], shiftOrigine, unghiuri)
        Head = self.schimbareSist(self.tblJoints[3], shiftOrigine, unghiuri)
        SpineShoulder = self.schimbareSist(self.tblJoints[20], shiftOrigine, unghiuri)

        return Neck, Head, SpineShoulder

    def head_tracking(self):
        referinta = (8, 4, 2)
        shift = self.setCartezian(referinta)
        unghiuri = (shift[0], shift[1], shift[2])
        shiftOrigine = (shift[3], shift[4], shift[5])
        Neck, Head, SpineShoulder = self.getJoints(shiftOrigine, unghiuri)

        # HeadYaw
        Ypoint = 10
        if Head[1] < 0:
            Ypoint = -10
        Angle = self.getAngle((Head[0], Head[1], 0), (Neck[0],
                                                      Neck[1], 0), (Neck[0], Ypoint, 0))
        if Head[0] > Neck[0]:
            Angle = -Angle
        HYangle = Angle

        # HeadPitch
        Angle = self.getAngle((0, Head[1], Head[2]), (0, Neck[1], Neck[2]), (0, SpineShoulder[1], SpineShoulder[2]))
        Angle = Angle - math.radians(150)  # pi
        HPangle = Angle
        math.degrees(HPangle)


if __name__ == '__main__':
    game = BodyGameRuntime()
    game.run()