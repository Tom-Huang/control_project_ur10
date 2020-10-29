#include "ros/ros.h"
#include <tf/transform_datatypes.h>

#include "leap_motion/Human.h"
#include "leap_motion/Hand.h"
#include "leap_motion/Finger.h"
#include "leap_motion/Bone.h"
#include "leap_motion/Gesture.h"
#include "leap_motion/Connect.h"
#include <iostream>

#include "../inc/lmc_listener.h"
#include "Leap.h"

using namespace Leap;

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

/*!
 * \brief Uses default values for initial setup.
 *
 * Called when a LeapListener object is created with this constructor.
 * Every gesture and ROS_INFO settings have been disabled.
 *
 */
LeapListener::LeapListener(){
    header_frame_id_ = "leap_hands";

    enable_controller_info_ = true;
    enable_frame_info_ = true;
    enable_hand_info_ = true;

    enable_gesture_circle_ = true;
    enable_gesture_swipe_ = false;
    enable_gesture_screen_tap_ = false;
    enable_gesture_key_tap_ = false;
}

/*!
 * \brief Uses the parameter server for initial setup values.
 *
 * Called when a LeapListener object is created with this constructor.
 * 
 * \param frame_id    The value that is set as the ros_human_msg.header.frame_id.
 * \param args[7]     Values that control info printing and gesture enabling.
 */
LeapListener::LeapListener(bool args[7]){
    header_frame_id_ = "leap_hands";

    enable_controller_info_ = args[0];
    enable_frame_info_ = args[1];
    enable_hand_info_ = args[2];

    enable_gesture_circle_ = args[3];
    enable_gesture_swipe_ = args[4];
    enable_gesture_screen_tap_ = args[5];
    enable_gesture_key_tap_ = args[6];
}

/*!
 * \brief Called when a connection is established with the controller.
 *
 * Called when a connection is established between the controller and the Leap Motion software, 
 * the controller calls the Listener::onConnect() function.
 * 
 * \param controller    The Controller object invoking this callback function. 
 */
void LeapListener::onConnect(const Controller& controller)
{
     
    if(LeapListener::enable_controller_info_)
    {   
        const DeviceList devices = controller.devices();
        for (int i = 0; i < devices.count(); ++i) 
        {
            ROS_INFO( "id: %s", devices[i].toString().c_str() );
            ROS_INFO("  isStreaming: %s", (devices[i].isStreaming() ? "true" : "false") );
        }
    }

    // A circular movement by a finger.
    if(LeapListener::enable_gesture_circle_)
    {
        controller.enableGesture(Leap::Gesture::TYPE_CIRCLE);
        if(LeapListener::enable_controller_info_)
        {
            ROS_INFO("Circle gesture has been enabled");
        }
    }

    // A straight line movement by the hand with fingers extended.
    if(LeapListener::enable_gesture_swipe_)
    {
        controller.enableGesture(Leap::Gesture::TYPE_SWIPE);
        if(LeapListener::enable_controller_info_)
        {
            ROS_INFO("Swipe gesture has been enabled");
        }
    }

    // A forward tapping movement by a finger. 
    if(LeapListener::enable_gesture_screen_tap_)
    {
        controller.enableGesture(Leap::Gesture::TYPE_SCREEN_TAP);
        if(LeapListener::enable_controller_info_)
        {
            ROS_INFO("Screen tap gesture has been enabled");
        }
    }

    // A downward tapping movement by a finger.
    if(LeapListener::enable_gesture_key_tap_)
    {
        controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);
        if(LeapListener::enable_controller_info_)
        {
            ROS_INFO("Key tap gesture has been enabled");
        }
    } 
};

/*!
 * \brief Called once, when this Listener object is newly added to a Controller.  
 *
 * When an instance of a Listener subclass is added to a Controller object, 
 * it calls the Listener::onInit() function when the listener is ready for use.
 * 
 * \param controller    The Controller object invoking this callback function.
 */
void LeapListener::onInit(const Controller& controller)
{ 
    if(LeapListener::enable_controller_info_)
    {
        ROS_INFO("Listener has been initialized");
        
    }
};

/*!
 * \brief Called when the Controller object disconnects 
 * 
 * Called when the Controller object disconnects from the Leap Motion software or 
 * the Leap Motion hardware is unplugged.
 * 
 * The controller can disconnect when the Leap Motion device is unplugged, 
 * the user shuts the Leap Motion software down, or the Leap Motion software 
 * encounters an unrecoverable error.
 * 
 * \param controller    The Controller object invoking this callback function.
 */
void LeapListener::onDisconnect(const Controller& controller)
{
    if(LeapListener::enable_controller_info_)
    {
        ROS_INFO("Device disconnected");
    }
};

/*!
 * \brief Called when this Listener object is removed from the Controller.
 * 
 * Called when this Listener object is removed from the Controller or 
 * the Controller instance is destroyed. 
 *
 * \param controller    The Controller object invoking this callback function.
 */
void LeapListener::onExit(const Controller& controller)
{
    if(LeapListener::enable_controller_info_)
    {
        ROS_INFO("Listener object removed from the device");
    }
};

/*!
 * \brief Called when a new frame of hand and finger tracking data is available.
 * 
 * Access the new frame data using the Controller::frame() function.
 * 
 * Note, the Controller skips any pending onFrame events while the onFrame handler executes. 
 * If the implementation takes too long to return, one or more frames can be skipped. 
 * The Controller still inserts the skipped frames into the frame history. 
 * You can access recent frames by setting the history parameter when calling 
 * the Controller::frame() function. You can determine if any pending onFrame events were skipped 
 * by comparing the ID of the most recent frame with the ID of the last received frame.
 * 
 * \param controller    The Controller object invoking this callback function.
 */
void LeapListener::onFrame(const Controller& controller) 
{   ros::NodeHandle nh2; // 声明一个节点句柄来与ROS系统进行通信
            //ROS_INFO("23333");
    ros::Publisher lm2ros_pub = nh2.advertise<leap_motion::Connect>("helloworld/lm2ros_msg", 1000);
    const Frame frame = controller.frame();
    ros::Time timestamp = ros::Time::now();
    
    // Create an instance of leap_motion::human message
    // and fill it with all the possible data from the Leap Motion controller
    leap_motion::Human ros_human_msg;
    leap_motion::Connect lm2ros_msg; // 以MsgTutorial消息文件格式声明一个 叫做msg的消息
    

    ros_human_msg.header.stamp = timestamp;
    ros_human_msg.header.frame_id = LeapListener::header_frame_id_;
    ros_human_msg.lmc_frame_id = frame.id();
    ros_human_msg.nr_of_hands = frame.hands().count();
    //frame.hands().count();从哪里来的？
    ros_human_msg.nr_of_fingers = frame.fingers().count();
    ros_human_msg.nr_of_gestures = frame.gestures().count();
    ros_human_msg.current_frames_per_second = frame.currentFramesPerSecond();
    ros_human_msg.to_string = frame.toString();

    ros_human_msg.right_hand.is_present = false;
    ros_human_msg.left_hand.is_present = false;

    //if(LeapListener::enable_frame_info_)
    //{
     // ROS_INFO("Frame id: %lu, timestamp: %f, hands: %i, fingers: %i, gestures: %i",
     // frame.id(), timestamp.toSec(), frame.hands().count(), frame.fingers().count(),
     // frame.gestures().count() );
    //}
    
    // Get a list of all of the hands detected in the frame
    HandList hand_list = frame.hands();



   // int translation;

   // int trigger = 0;
 
    float a,b,c,x0=0,x,y0=0.2,y,z0=0,z;

    int grabbed;
 
   // int trigger_old;
 
    int translation_value;
    if(hand_list.count() > 2)
    {
        ROS_WARN("There are more than 2 hands in the frame! I see %i hands.", hand_list.count());
    }
    else
    {   //ROS_INFO("2.5");
        // if(frame.hands().count()==2)
        //     {
        //             ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //         ROS_INFO("Before:trigger:%d trigger_old:%d",trigger,trigger_old); 
        //         if (frame.gestures().count()!=0) 
        //         {trigger=1 ;} 
        //         ROS_INFO("After:trigger:%d trigger_old:%d",trigger,trigger_old); 
        //         if(trigger_old==0 && trigger == 1)
        //         {translation = 1;}
        //         ROS_INFO("2.8: translation:%d",translation);
        //         //ROS_INFO("Type: %d",lm2ros_msg.is_translation);   
        //         lm2ros_msg.is_roll=0;
        //         lm2ros_msg.is_pitch=0;
        //         lm2ros_msg.is_yaw=0;
        //         lm2ros_msg.is_forward=0;
        //         lm2ros_msg.is_up=0;
        //         lm2ros_msg.is_right=0;
        //         trigger_old=trigger;
        //         ROS_INFO("End: trigger_old:%d",trigger_old); 
        //     }
        for (HandList::const_iterator current_hand = hand_list.begin(); current_hand != hand_list.end(); ++current_hand)
        {
            // Get a hand from the list
            const Hand hand = *current_hand;

            // Create an instance of leap_motion::Hand message
            // and fill it with all the possible data from the Leap::Hand object
            leap_motion::Hand ros_hand_msg;
            ros_hand_msg.header.stamp = timestamp;
            ros_hand_msg.header.frame_id = LeapListener::header_frame_id_;
            ros_hand_msg.lmc_hand_id = hand.id();
            ros_hand_msg.is_present = true;                     // Override the default value
            ros_hand_msg.valid_gestures = false;                // No gestures associated with this hand
            ros_hand_msg.confidence = hand.confidence();        // How confident the controller is with a given hand pose between [0,1] inclusive. 
            
            // Get hand's roll-pitch-yam and convert them into quaternion.
            // NOTE: Leap Motion roll-pith-yaw is from the perspective of human.
            // Here it is mapped that roll is about x-, pitch about y-, and yaw about z-axis.
            ros_hand_msg.roll = hand.direction().pitch();         // The roll angle in radians. 
            ros_hand_msg.pitch = hand.direction().yaw();          // The pitch angle in radians.
            ros_hand_msg.yaw = hand.palmNormal().roll();          // The yaw angle in radians.
             
            //important
            ros_hand_msg.grab_strength = hand.grabStrength();     // The angle between the fingers and the hand of a grab hand pose. 
            ros_hand_msg.palm_width = hand.palmWidth() / 1000.0;  // in m
            ros_hand_msg.pinch_strength = hand.pinchStrength();   // The distance between the thumb and index finger of a pinch hand pose.
            ros_hand_msg.time_visible = hand.timeVisible();       // The duration (in seconds) that this Hand has been tracked. 
            ros_hand_msg.to_string = hand.toString();

            // The rate of change of the palm position (x,y,z) in m/s.
            ros_hand_msg.palm_velocity.push_back(hand.palmVelocity()[0] / 1000.0); 
            ros_hand_msg.palm_velocity.push_back(hand.palmVelocity()[1] / 1000.0);
            ros_hand_msg.palm_velocity.push_back(hand.palmVelocity()[2] / 1000.0);

            // The center of a sphere fit to the curvature of this hand in m.
            ros_hand_msg.sphere_center.push_back(hand.sphereCenter()[0] / 1000.0); 
            ros_hand_msg.sphere_center.push_back(hand.sphereCenter()[1] / 1000.0); 
            ros_hand_msg.sphere_center.push_back(hand.sphereCenter()[2] / 1000.0);

            ros_hand_msg.sphere_radius = hand.sphereRadius() / 1000.0;   // in m

            // The position of the wrist of this hand in m.
            ros_hand_msg.wrist_position.push_back(hand.wristPosition()[0] / 1000.0);
            ros_hand_msg.wrist_position.push_back(hand.wristPosition()[1] / 1000.0);
            ros_hand_msg.wrist_position.push_back(hand.wristPosition()[2] / 1000.0);
 
            
            //important//important   
            // The center position of the palm in meters from the Leap Motion Controller origin. 
            ros_hand_msg.palm_center.x = hand.palmPosition().x / 1000.0;
            ros_hand_msg.palm_center.y = hand.palmPosition().y / 1000.0;
            ros_hand_msg.palm_center.z = hand.palmPosition().z / 1000.0;
            
            //if(LeapListener::enable_hand_info_)
            //{    //show which hand and the position
             
               // ROS_INFO("%s %s, id: %i, palm position: %s mm", std::string(2, ' ').c_str(), 
               // (hand.isRight() ? "Right hand" : "Left hand"), hand.id(), hand.palmPosition().toString().c_str() ); 
                //ROS_INFO("Roll:%f ; Pitch:%f ; Yaw:%f ",ros_hand_msg.roll,ros_hand_msg.pitch, ros_hand_msg.yaw ); 
           // }
            // This is a list of finger messages that will be attached to the hand message
            std::vector<leap_motion::Finger> finger_msg_list; 

            // Given in order from thumb[idx 0] to pinky[idx 4].
            const FingerList fingers = hand.fingers();
            for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) 
            {
                // Get a finger from the list
                const Finger finger = *fl;

                leap_motion::Finger ros_finger_msg;
                ros_finger_msg.header.stamp = timestamp;
                ros_finger_msg.header.frame_id = LeapListener::header_frame_id_;
                ros_finger_msg.lmc_finger_id = finger.id();
                ros_finger_msg.type = finger.type();
                ros_finger_msg.length = finger.length() / 1000.0;
                ros_finger_msg.width = finger.width() / 1000.0;
                ros_finger_msg.to_string = finger.toString();

                if(LeapListener::enable_hand_info_)
                {
                    //ROS_INFO("%s %s, id: %i, length: %f mm, width: %f mm", std::string(4, ' ').c_str(), 
                    //fingerNames[finger.type()].c_str(), finger.id(), finger.length(), finger.width()); 
                }

                Bone bone;
                Bone::Type boneType;
                std::vector<leap_motion::Bone> bone_msg_list;
                for (int b = 0; b < 4; ++b) 
                {
                    // Get current fingers bone at index b
                    boneType = static_cast<Bone::Type>(b);
                    bone = finger.bone(boneType);

                    leap_motion::Bone ros_bone_msg;
                    ros_bone_msg.type = boneType;
                    ros_bone_msg.length = bone.length() / 1000.0; // in m
                    ros_bone_msg.width = bone.width() / 1000.0;   // in m
                    ros_bone_msg.to_string = bone.toString();     // human readable description of the Bone object
                    
                    ros_bone_msg.center.push_back(bone.center()[0] / 1000.0); // x in m
                    ros_bone_msg.center.push_back(bone.center()[1] / 1000.0); // y in m
                    ros_bone_msg.center.push_back(bone.center()[2] / 1000.0); // z in m

                    ros_bone_msg.bone_start.position.x = bone.prevJoint().x / 1000.0;
                    ros_bone_msg.bone_start.position.y = bone.prevJoint().y / 1000.0;
                    ros_bone_msg.bone_start.position.z = bone.prevJoint().z / 1000.0;
                    
                    ros_bone_msg.bone_end.position.x = bone.nextJoint().x / 1000.0;
                    ros_bone_msg.bone_end.position.y = bone.nextJoint().y / 1000.0;
                    ros_bone_msg.bone_end.position.z = bone.nextJoint().z / 1000.0;

                    bone_msg_list.push_back(ros_bone_msg);
                }
                
                ros_finger_msg.bone_list = bone_msg_list;
                finger_msg_list.push_back(ros_finger_msg);
            }
            
            ros_hand_msg.finger_list = finger_msg_list;
            
            // Check if there are any gestures assosciated wih this frame
            // if there are, connect them with the correct hand
            if (!frame.gestures().isEmpty())
            {
                std::vector<leap_motion::Gesture> gesture_msg_list; 
                ros_hand_msg.valid_gestures = true;

                Leap::GestureList gestures = frame.gestures();
                for(Leap::GestureList::const_iterator gl = gestures.begin(); gl != gestures.end(); gl++)
                {
                    // Check what hands are connected with a particular gesture
                    Leap::HandList hands_for_gesture = (*gl).hands();
                    for(unsigned int i = 0; i < hands_for_gesture.count(); i++)
                    {
                        if(*current_hand == hands_for_gesture[i])
                        {
                            leap_motion::Gesture ros_gesture_msg;
                            ros_gesture_msg.lmc_gesture_id = (*gl).id();
                            ros_gesture_msg.duration_us = (*gl).duration();
                            ros_gesture_msg.duration_s = (*gl).durationSeconds();
                            ros_gesture_msg.is_valid = (*gl).isValid();
                            ros_gesture_msg.gesture_state = (*gl).state();
                            ros_gesture_msg.gesture_type = (*gl).type();
                            ros_gesture_msg.to_string = (*gl).toString();
                            
                            // Appends the ids of pointable objects related with this gesture
                            Leap::PointableList pointables_for_gesture = (*gl).pointables();
                            std::vector<int32_t> pointable_ids;
                            for(unsigned int j = 0; j < pointables_for_gesture.count(); j++)
                            {
                                pointable_ids.push_back( pointables_for_gesture[j].id() );
                            }
                            
                            ros_gesture_msg.pointable_ids = pointable_ids;
                            gesture_msg_list.push_back(ros_gesture_msg);
                        }
                    }
                }
                ros_hand_msg.gesture_list = gesture_msg_list;
            }


            //ROS_INFO("2");
            
            //leap_motion::Connect lm2ros_msg; // 以MsgTutorial消息文件格式声明一个 叫做msg的消息
            int count;
            if(LeapListener::enable_hand_info_)
            {    //show which hand and the position
                
            //ROS_INFO("%s %s, id: %i, palm position: %s mm", std::string(2, ' ').c_str(), 
               // (hand.isRight() ? "Right hand" : "Left hand"), hand.id(), hand.palmPosition().toString().c_str() ); 
            //ROS_INFO("Roll:%d ; Pitch:%d ; Yaw:%d ",lm2ros_msg.is_roll,lm2ros_msg.is_pitch, lm2ros_msg.is_yaw ); 
            
            //else if (ros_gesture_msg.gesture_type == 1) {
            //  lm2ros_msg.is_translation=0 ; 
            // }
            
            //ROS_INFO("3,%d",count);
            //count++;
            if(frame.hands().count()==2)
            {
                lm2ros_msg.is_twohands = 1;
            }
            else  lm2ros_msg.is_twohands = 0;
            
            if(hand.isLeft()&&(frame.hands().count()==1))
            {   //ROS_INFO("Frame id: %lu,hands: %i, fingers: %i, hand: %f",
                //frame.id(), frame.hands().count(), frame.fingers().count(),hand.grabStrength());
                //ROS_INFO("4");
                ROS_INFO("CHANGING THE MODE:%d",translation_value);
                if(hand.grabStrength()!=1)
                {
                    translation_value = 0;
                    lm2ros_msg.is_translation  = translation_value;
                }
                if(hand.grabStrength()==1)
                {
                    translation_value = 1;
                    lm2ros_msg.is_translation = translation_value;
                    //ROS_INFO("Grabbed");
                }
                //ROS_INFO("Grabbed or not: %d", grabbed); 
            } 

            //move the robot
    
            if(hand.isRight()&&(frame.hands().count()==1))  
                {   
                    //ROS_INFO("5");
                    if(hand.grabStrength()==1)
                    {   
                        ROS_INFO("//////////Gripper is CLOSED//////////");
                        grabbed=1;
                    }
                if(hand.grabStrength()!=1)
                    {   
                        grabbed=0;
                    }
                    //ROS_INFO("6");
                    ROS_INFO("MOVING THE ROBOT."); 
                    //ROS_INFO("Grabbed or not: %d", lm2ros_msg.is_grab); 
                    a = hand.direction().pitch();         // The roll angle in radians. 
                    b = hand.direction().yaw();          // The pitch angle in radians.
                    c = hand.palmNormal().roll();          // The yaw angle in radians.
                    x = hand.palmPosition().x / 1000.0;
                    y = hand.palmPosition().y / 1000.0;
                    z = hand.palmPosition().z / 1000.0; 
                    //revolution
                    //ROS_INFO("Move type:%d",translation_value);
                    if(translation_value == 0)
                    {
                        x = 0;
                        y = 0.2;
                        z = 0;
                    }
                    if(translation_value == 1)
                    {
                        a = 0;
                        b = 0;
                        c = 0;
                    }
                    //ROS_INFO("roll: %f; pitch: %f; yaw: %f; x: %f; y: %f; z:%f",a,b,c,x,y,z); 
                    //ROS_INFO("roll: %f; pitch: %f; yaw: %f; x: %f; y: %f; z:%f",a,b,c,x-x0,y-y0,z-z0);  
                    ROS_INFO("*****************COMMAND*******************");
                    if ((a)<-0.3) {lm2ros_msg.is_roll = 1;ROS_INFO("positive roll"); }
                    else if ((a)>0.3) {lm2ros_msg.is_roll = -1;ROS_INFO("negative roll");} 
                    else {lm2ros_msg.is_roll = 0;}

                    if ((b)<-0.3) {lm2ros_msg.is_pitch = 1;ROS_INFO("positive pitch"); }
                    else if ((b)>0.3) {lm2ros_msg.is_pitch = -1;ROS_INFO("negative pitch"); }
                    else lm2ros_msg.is_pitch = 0;

                    if ((c)<-0.3) {lm2ros_msg.is_yaw = 1;ROS_INFO("positive yaw"); }
                    else if ((c)>0.3) {lm2ros_msg.is_yaw = -1;ROS_INFO("negative yaw"); }
                    else lm2ros_msg.is_yaw = 0;

                    //tranlation
                    if ((x-x0)>0.05)  {lm2ros_msg.is_right = 1;ROS_INFO("go right"); }
                    else if ((x-x0)<-0.05)  {lm2ros_msg.is_right = -1;ROS_INFO("go left"); }
                    else lm2ros_msg.is_right = 0;

                    if ((y-y0)>0.05)  {lm2ros_msg.is_up = 1;ROS_INFO("go up"); }
                    else if ((y-y0)<-0.05)  {lm2ros_msg.is_up = -1;ROS_INFO("go down"); }
                    else lm2ros_msg.is_up = 0;

                    if ((z-z0)>0.05) {lm2ros_msg.is_forward = -1;ROS_INFO("go back"); }
                    else if ((z-z0)<-0.05) {lm2ros_msg.is_forward = 1;ROS_INFO("go forward"); }
                    else lm2ros_msg.is_forward = 0;
                    ROS_INFO("*******************************************");
                }
            
            }
            
            if( hand.isRight() )
            {
                ros_human_msg.right_hand = ros_hand_msg;
            }
            else
            {
                ros_human_msg.left_hand = ros_hand_msg;
            }
        }
        //ROS_INFO("Grabbed:%d",grabbed);
        lm2ros_msg.is_grab  = grabbed;
        //ROS_INFO("Transform or not?:%d",translation);
        //ROS_INFO("Transform Value before:%d",translation_value);
        //if (translation==1) {translation_value = !translation_value ;}
        
        //lm2ros_msg.is_translation=translation_value ;
        lm2ros_msg.is_translation = translation_value;
        
        ROS_INFO("mode:%d  gripper:%d  position:(%d %d %d)  revolution(%d %d %d) 2hands:%d",lm2ros_msg.is_translation,lm2ros_msg.is_grab,
        lm2ros_msg.is_forward,lm2ros_msg.is_right,lm2ros_msg.is_up,
        lm2ros_msg.is_roll,lm2ros_msg.is_pitch,lm2ros_msg.is_yaw,lm2ros_msg.is_twohands);
        
        lm2ros_pub.publish(lm2ros_msg);
        ros_publisher.publish(ros_human_msg); 
    }
};