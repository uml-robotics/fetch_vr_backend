#!/usr/bin/env python
import Tkinter as tk
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import tf
import threading
from datetime import datetime
from datetime import timedelta
import json
import argparse
import os
import subprocess
import rospkg

rospy.init_node('experimenter_ui', anonymous=True)
rospack = rospkg.RosPack()

config_path = rospy.get_param('/experimenter/user_study/configpath',self.rospack.get_path('fetch_vr_backend')+"/config/participant_configs.json")

#parser = argparse.ArgumentParser(description="The following parameters are used in this file: ",
#                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)

#parser.add_argument("-f", "--filename",  help="JSON File for participant configs",  required=True)

#args = parser.parse_args()
with open(config_path) as f:
    try:
        json.load(f)
    except Exception as e:
        print(e)

# CONSTANTS
ROS_PREFIX = "user_study/"


RUN_TYPE = [
"Navigation",
"Manipulation"
]

RUN_ID = [
1,
2,
3
]


SA1_OPTIONS = [
"Where is the obstacle closest to the robot?",
"What color is the obstacle closest to the robot?",
"Where is the robot on this map?",
"Which wall is closest to the robot?"
]

SA2_OPTIONS = [
"How many objectives has the robot reached thus far?",
"Where was the last objective the robot reached? If you have not yet reached an objective, where was the starting location?",
"Out of the three objectives, how many are remaining?",
"Where is the next objective?"
]

SA3_OPTIONS = [
"Based on where the robot is currently located on the grid, if the robot drives up 2 grid blocks, will it run into any obstacles?",
"Based on where the robot is currently located on the grid, if the robot drives down 2 grid blocks, will it run into any obstacles.",
"Based on where the robot is currently located on the grid, if the robot drives right 2 grid blocks, will it run into any obstacles.",
"In which direction would the robot need to turn toward to be at the next target?",
"In terms of distance, what percent of the way are you to your next target? Please use the slider."
]

OBSTACLE_COLORS = [
    "orange;yellow;grey",
    "grey;orange;yellow",
    "yellow;grey;orange"
]

# GLOBALS
id = "0"
isStarted = False
startTime = -1
isPaused = False
pauseStartTime = -1
totalPauseDuration = timedelta(0)
isSAStarted = False
saCallback = None
saCurrentDelay = -1
saStartTime = -1

startBtn = None
pauseBtn = None
#saBtn = None
saLabel = None
timeLabel = None
saTimeLabel = None

currentRunLabel = None
runTypeSelect = None
runType = None
runIDSelect = None
runID = None

runConfigs = None
raw_config = ''
interface = ''

currentRunType = "None"
currentRunID = -1
currentSAIndex = 0
answersReceived = 0
arena_config = -1
saQuestionIndex = -1

process = None
bag_directory = "/home/csrobot/experimentdata"

def answer_cb(answer):
    global answersReceived
    answersReceived += 1
    if answersReceived != 3:
        return
    unPause()
    if currentSAIndex < 6:
        onStartSAPressed()
    

# ROS

pause_pub = rospy.Publisher(ROS_PREFIX + 'pause', Bool, queue_size=10)
start_pub = rospy.Publisher(ROS_PREFIX + 'start', Bool, queue_size=10)
end_pub = rospy.Publisher(ROS_PREFIX + 'stop', Bool, queue_size=10)
time_pub = rospy.Publisher(ROS_PREFIX + 'runtime', Header, queue_size=10)
question_pub = rospy.Publisher(ROS_PREFIX + 'sa/' + 'question', String, queue_size=10)
transform_pub = rospy.Publisher(ROS_PREFIX + 'sa/' + 'tracked_poses', TransformStamped, queue_size=10)
run_number_pub = rospy.Publisher(ROS_PREFIX +'runnumber', Int32, queue_size=10)
run_type_pub = rospy.Publisher(ROS_PREFIX + 'runtype', String, queue_size=10)
participant_id_pub = rospy.Publisher(ROS_PREFIX + 'participant_id', String, queue_size=10)

answer_sub = rospy.Subscriber(ROS_PREFIX + 'sa/' + 'answer', String, answer_cb)

file_path = rospy.get_param('/experimenter/user_study/logpath',self.rospack.get_path('fetch_vr_backend')+"/data")

listener = tf.TransformListener()
rate = rospy.Rate(10)

def update():
    if isStarted:
        totalElapsedTime = datetime.now()-startTime

        currentPauseElapsedTime = timedelta(0)
        if isPaused:
            currentPauseElapsedTime = datetime.now() - pauseStartTime

        elapsedTime = totalElapsedTime - currentPauseElapsedTime - totalPauseDuration

        timeLabel.config(text=str(elapsedTime).split(".")[0])
        timeMsg = Header()
        timeMsg.stamp.secs = elapsedTime.total_seconds()
        time_pub.publish(timeMsg)

        if currentRunType is not None:
            msg = String()
            msg.data = currentRunType
            run_type_pub.publish(msg)
        
        if currentRunID is not -1:
            msg = Int32()
            msg.data = currentRunID
            run_number_pub.publish(msg)

        if id is not "0":
            msg = String()
            msg.data = id
            participant_id_pub.publish(msg)

        if isSAStarted:
            if(isPaused):
                elapsedSATime = timedelta(seconds=0)
            else:
                elapsedSATime = datetime.now() - saStartTime

            countdown = timedelta(seconds=saCurrentDelay) - elapsedSATime

            saTimeLabel.config(text=str(countdown).split(".")[0])
    window.after(100, lambda: update())

def startBag():
    global process
    bag_name = bag_directory + "P" + id + "_"+currentRunType + "_" + str(currentRunID)
    process = subprocess.Popen(["rosbag", "record", "--output-name="+bag_name, "/tf","/tf_static","/base_scan","/head_camera/rgb/image_raw/compressed", "-e", "/user_study.*", "-e","/move_group.*", "-e","/move_base.*","-e","/head_controller.*","-e","/arm_with_torso_controller.*","-e","","-e","/gripper_controller.*","-e","/fetch_rviz_interface.*"])
    

def stopBag():
    global process
    #print ("Stopping bag recording")
    process.terminate()
    return

def onPausePressed():
    if not isStarted:
        return

    if isPaused:
        unPause()
    else:
        pause()

    pause_pub.publish(isPaused)
    rate.sleep()

def pause():
    global isPaused
    isPaused = True
    pauseBtn.configure(text="RESUME")
    global pauseStartTime
    pauseStartTime = datetime.now()
    if isSAStarted:
        pauseSATimer()

def unPause():
    global isPaused
    isPaused = False
    pauseBtn.configure(text="PAUSE")
    global totalPauseDuration
    global pauseStartTime
    totalPauseDuration += (datetime.now() - pauseStartTime)
    pauseStartTime = -1
    if isSAStarted:
        unPauseSATimer()

def onStartPressed():
    if isStarted:
        endRun()
    else:
        startRun()

def startRun():
    global isStarted
    isStarted = True
    global startTime
    startTime = datetime.now()

    startBtn.configure(text="END RUN")

    currentRunLabel.pack()
    global currentRunType
    global currentRunID
    currentRunLabel.config(text=runType.get() + " Run: " + str(runID.get()))
    currentRunType = runType.get()
    currentRunID = runID.get()
    runTypeSelect.pack_forget()
    runIDSelect.pack_forget()

    saTimeLabel.configure(text="")

    onStartSAPressed()
    startBag()
    start_pub.publish(True)
    rate.sleep()

def endRun():
    global isStarted
    isStarted = False
    global startTime
    startTime = -1

    global currentRunType
    currentRunType = "None"
    global currentRunID
    currentRunID = -1
    global currentSAIndex
    currentSAIndex = 0
    global answersReceived
    answersReceived = 0

    stopBag()

    # RESET PAUSE
    if isPaused:
        unPause()
    global totalPauseDuration
    totalPauseDuration = timedelta(0)

    # RESET SA
    if isSAStarted:
        clearSATimer()

    startBtn.configure(text="START RUN")
    timeLabel.configure(text='')

    currentRunLabel.pack_forget()
    runTypeSelect.pack()
    runIDSelect.pack()


    end_pub.publish(True)

def onStartSAPressed():
    if not isStarted or isPaused:
        saTimeLabel.config(text='CAN NOT RUN SA WHEN NOT IN A RUN!')
        return

    if currentRunType == "Manipulation":
        saTimeLabel.config(text='CAN NOT RUN SA FOR MANIP!')
        return

    if currentSAIndex >= 6:
        saTimeLabel.config(text='CAN NOT RUN MORE THAN 6 SA PER RUN!')
        return

    global answersReceived
    answersReceived = 0
    
    #if isSAStarted:
    #    clearSATimer()
    #else:
    startSATimer()

def startSATimer():
    global isSAStarted
    isSAStarted = True
    #saBtn.configure(text="CANCEL SA TIMER")

    global arena_config
    arena_config = -1
    q_config = None
    for val in runConfigs:
        normalizedRunID = val['id']
        if normalizedRunID > 3:
            normalizedRunID = normalizedRunID - 3
        if val['type'] == currentRunType and normalizedRunID == currentRunID:
            arena_config = val['arena_config']
            for question_set in val['question']:
                if question_set['id'] == (currentSAIndex + 1):
                    q_config = question_set
                    break
        if q_config:
            break
    print(q_config)

    global saCurrentDelay
    saCurrentDelay = q_config['time']
    global saCallback
    global saQuestionIndex
    saQuestionIndex = q_config['index'] - 1
    saCallback = threading.Timer(saCurrentDelay, askSA, [saQuestionIndex, saQuestionIndex, saQuestionIndex, arena_config])
    saCallback.start()
    global saStartTime
    saStartTime = datetime.now()

def pauseSATimer():
    global saCallback
    if(saCallback):
        saCallback.cancel()
    totalTimeRun = pauseStartTime - saStartTime
    global saCurrentDelay
    saCurrentDelay -= totalTimeRun.total_seconds()

def unPauseSATimer():
    global saCallback
    saCallback = threading.Timer(saCurrentDelay, askSA, [saQuestionIndex, saQuestionIndex, saQuestionIndex, arena_config])
    saCallback.start()
    global saStartTime
    saStartTime = datetime.now()

def clearSATimer():
    global isSAStarted
    isSAStarted = False
    #saBtn.configure(text="START SA TIMER")
    global saCurrentDelay
    saCurrentDelay = -1
    global saCallback
    if(saCallback):
        saCallback.cancel()
    global saStartTime
    saStartTime = -1

    saTimeLabel.config(text='')

def askSA(q1, q2, q3, arena): # keeping 3 arguments so we can change back easily if needed
    pause()
    global currentSAIndex
    rospy.set_param('/user_study/participant_id', id)
    rospy.set_param('/user_study/run_number', currentRunID)
    rospy.set_param('/user_study/interrupt_number', currentSAIndex + 1)

    questionStr = SA1_OPTIONS[q1] + ";" + SA2_OPTIONS[q2] + ";" + SA3_OPTIONS[q3]  #sa1.get() + ";" + sa2.get() + ";" + sa3.get()

    clearSATimer()

    currentSAIndex = currentSAIndex + 1
    global saLabel
    saLabel.configure(text="Number of SA asked: " + str(currentSAIndex))
    obstacle_config = OBSTACLE_COLORS[currentRunID - 1].split(";")

    publishTransform('map', 'base_link')
    publishTransform('base_link', 'target_a')
    publishTransform('base_link', 'target_b')
    publishTransform('base_link', 'target_c')
    publishTransform('base_link', 'obstacle_a', obstacle_config[0])
    publishTransform('base_link', 'obstacle_b', obstacle_config[1])
    publishTransform('base_link', 'obstacle_c', obstacle_config[2])

    print("ASKING: " + questionStr)
    question_pub.publish(questionStr)
    rate.sleep()

def publishTransform(_from, _to, new_name=""):
    if new_name == "":
        new_name = _to
    try:
        tf = listener.lookupTransform(_from, _to, rospy.Time(0))
        tf_stamped = TransformStamped()
        tf_stamped.header.frame_id = _from
        tf_stamped.child_frame_id = new_name
        tf_stamped.transform.translation.x = tf[0][0]
        tf_stamped.transform.translation.y = tf[0][1]
        tf_stamped.transform.translation.z = tf[0][2]
        tf_stamped.transform.rotation.x = tf[1][0]
        tf_stamped.transform.rotation.y = tf[1][1]
        tf_stamped.transform.rotation.z = tf[1][2]
        tf_stamped.transform.rotation.w = tf[1][3]
        transform_pub.publish(tf_stamped)
    except Exception as e:
            print(e)

def onIDConfirmed():
    global id
    id = idInput.get(1.0, "end-1c")
    id = id.strip()

    with open(args.filename) as f:
        data = json.load(f)
## For Debug
#        print(json.dumps(data, indent=4, sort_keys=True))

    isValidID = False
    for key, value in data.items():
        for term in value:
            if term['id'] == id:
               global runConfigs
               runConfigs = term['run']
               global raw_config
               raw_config = term['raw_config']
               global interface
               interface = term['interface']
               isValidID = True
               break
        if isValidID:
            break

    if isValidID:
        loadExperimenterUI()
    else:
        participantIDLabel.configure(text="INVALID ID: " + str(id))
        idInput.delete("1.0","end")

def loadExperimenterUI():
    for i in range(3):
        window.columnconfigure(i, weight=1, minsize=480)
        window.rowconfigure(i, weight=1, minsize=480)

    frame = tk.Frame(
        master=window,
        relief=tk.RAISED,
        borderwidth=1
    )

    frame.grid(row=0, column=0, padx=5, pady=5)
    currentIDLabel = tk.Label(master=frame, text="Interface: " + interface + "\nRun Order: " + raw_config + '\nParticipant ID: ' + id) 
    currentIDLabel.pack(padx=5, pady=5)

    global startBtn
    startBtn = tk.Button(master=frame, text="START", width=30,
                      command=onStartPressed)
    startBtn.pack(padx=5, pady=5)
    global timeLabel
    timeLabel = tk.Label(master=frame, text='')
    timeLabel.pack(padx=5, pady=5)

    global currentRunLabel
    currentRunLabel = tk.Label(master=frame, text='')

    global runID
    runID = tk.IntVar(frame)
    global runType
    runType = tk.StringVar(frame)
    for run in runConfigs:
        if run['id'] == 1:
            runType.set(RUN_TYPE[run['type'] == 'Manipulation'])
            runID.set(RUN_ID[int(run['arena_config']) - 1])
            break

    global runTypeSelect
    runTypeSelect = tk.OptionMenu(frame, runType, *RUN_TYPE)
    runTypeSelect.pack()

    global runIDSelect
    runIDSelect = tk.OptionMenu(frame, runID, *RUN_ID)
    runIDSelect.pack()


    frame = tk.Frame(
        master=window,
        relief=tk.RAISED,
        borderwidth=1
    )

    frame.grid(row=0, column=2, padx=5, pady=5)
    global pauseBtn
    pauseBtn = tk.Button(master=frame, text="PAUSE", width=30,
                      command=onPausePressed)
    pauseBtn.pack(padx=5, pady=5)

    frame = tk.Frame(
        master=window,
        relief=tk.RAISED,
        borderwidth=1
    )

    frame.grid(row=1, column=0, padx=5, pady=5)
    global saLabel
    saLabel = tk.Label(master=frame, text='Number of SA asked: 0')
    saLabel.pack(padx=5, pady=5)
    #global saBtn
    #saBtn = tk.Button(master=frame, text="START SA TIMER", width=30,
    #                  command=onStartSAPressed)
    #saBtn.pack(padx=5, pady=5)
    global saTimeLabel
    saTimeLabel = tk.Label(master=frame, text='')
    saTimeLabel.pack(padx=5, pady=5)

    global start_frame
    start_frame.destroy()

    update()

# Main Loop for TK
window = tk.Tk()

window.columnconfigure(1, weight=1, minsize=480)
window.rowconfigure(1, weight=1, minsize=480)


start_frame = tk.Frame(
    master=window,
    relief=tk.RAISED,
    borderwidth=1
)
start_frame.grid(row=0, column=0, padx=5, pady=5)

participantIDLabel = tk.Label(master=start_frame, text='Please enter the participant id below:')
participantIDLabel.pack(padx=5, pady=5)
idInput = tk.Text(start_frame, height = 1, width = 20)
idInput.pack()
confirmIDBtn = tk.Button(master=start_frame, text="Confirm ID", width=30,
                  command=onIDConfirmed)
confirmIDBtn.pack(padx=5, pady=5)

window.mainloop()
rospy.spin()
