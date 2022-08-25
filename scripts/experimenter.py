#!/usr/bin/env python
import Tkinter as tk
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf
import threading
from datetime import datetime
from datetime import timedelta
import json
import argparse

parser = argparse.ArgumentParser(description="The following parameters are used in this file: ",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)

parser.add_argument("-f", "--filename",  help="JSON File for participant configs",  required=True)

args = parser.parse_args()
with open(args.filename) as f:
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

DELAY_OPTIONS = [
90,
135,
180
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
id = -1
isStarted = False
startTime = -1
continueTime = -1
isPaused = False
isReset = False
shouldRestartSA = False
pauseStartTime = -1
totalPauseDuration = timedelta(0)
totalResetDuration = timedelta(0)
activeCheckPointPauseDuration = timedelta(0)
activeCheckPointResetDuration = timedelta(0)
isSAStarted = False
saCallback = None
saCurrentDelay = -1
saStartTime = -1

startBtn = None
pauseBtn = None
resetBtn = None
checkpointBtn = None
timeLabel = None
saTimeLabel = None
resetLabel = None
timeSplitLabel = None
sa1Select = None
sa2Select = None
sa3Select = None
delaySelect = None
delay = None
sa1 = None
sa2 = None
sa3 = None

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

arena_config = -1
saQuestionIndex = -1

checkpoints = []

# ROS
rospy.init_node('experimenter_ui', anonymous=True)
pause_pub = rospy.Publisher('pause', Bool, queue_size=10)
start_pub = rospy.Publisher('start', Bool, queue_size=10)
end_pub = rospy.Publisher('stop', Bool, queue_size=10)
time_pub = rospy.Publisher('runtime', Header, queue_size=10)
question_pub = rospy.Publisher('question', String, queue_size=10)
transform_pub = rospy.Publisher(ROS_PREFIX + 'tracked_poses', TransformStamped, queue_size=10)
listener = tf.TransformListener()
rate = rospy.Rate(10)

def update():
    if isStarted:
        currentTime = datetime.now()
        totalElapsedTime = currentTime-startTime
    
        currentPauseElapsedTime = timedelta(0)
        if isPaused:
            currentPauseElapsedTime = currentTime - pauseStartTime

        elapsedTime = totalElapsedTime - currentPauseElapsedTime - totalPauseDuration - totalResetDuration

        timeLabel.config(text=str(elapsedTime).split(".")[0])
        timeMsg = Header()
        timeMsg.stamp.secs = elapsedTime.total_seconds()
        time_pub.publish(timeMsg)
        if isSAStarted:
            if isPaused or isReset:
                elapsedSATime = timedelta(seconds=0)
            else:
                elapsedSATime = currentTime - saStartTime

            countdown = timedelta(seconds=saCurrentDelay) - elapsedSATime

            saTimeLabel.config(text=str(countdown).split(".")[0])
    window.after(100, lambda: update())

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
    currentTime = datetime.now()

    global isReset
    if isReset:
        isReset = False
        global shouldRestartSA
        if shouldRestartSA:
            shouldRestartSA = False
            startSATimer()

    global totalPauseDuration
    global pauseStartTime
    global activeCheckPointPauseDuration
    activeCheckPointPauseDuration += (currentTime - pauseStartTime)
    totalPauseDuration += (currentTime - pauseStartTime)
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
    global continueTime
    continueTime = startTime

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

    start_pub.publish(True)

    onCheckpointReached()    
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

    # RESET PAUSE
    if isPaused:
        unPause()
    global totalPauseDuration
    totalPauseDuration = timedelta(0)
    global activeCheckPointPauseDuration
    activeCheckPointPauseDuration = timedelta(0)

    # RESET SA
    if isSAStarted:
        clearSATimer()

    # RESET CHECKPOINTS
    global isReset
    isReset = False
    global checkpoints
    checkpoints = []
    resetLabel.configure(text="")
    timeSplitLabel.configure(text="")
    global totalResetDuration
    totalResetDuration = timedelta(0)
    global activeCheckPointResetDuration
    activeCheckPointResetDuration = timedelta(0)
    global continueTime
    continueTime = -1

    startBtn.configure(text="START RUN")
    timeLabel.configure(text='')

    currentRunLabel.pack_forget()
    runTypeSelect.pack()
    runIDSelect.pack()


    end_pub.publish(True)

def onCheckpointReached():
    if not isStarted or isPaused:
        saTimeLabel.config(text='CAN NOT RUN SA WHEN NOT IN A RUN!')
        return

    if currentSAIndex >= 3:
        saTimeLabel.config(text='CAN NOT RUN MORE THAN 3 SA PER RUN!')
        return

    global continueTime

    if currentRunType == "Manipulation":
        if len(checkpoints) > 3:
            saTimeLabel.config(text='CAN NOT RUN MORE THAN 3 CHECKPOINTS PER RUN!')
            return
        if not isReset:
            continueTime = datetime.now()
            addCheckpoint(continueTime-startTime)
        return

    if isSAStarted:
        clearSATimer()
    else:
        startSATimer()
        if not isReset:
            continueTime = saStartTime
            addCheckpoint(continueTime-startTime)

def startSATimer():
    global isSAStarted
    isSAStarted = True
    if currentRunType == "Navigation":
        checkpointBtn.configure(text="CANCEL SA TIMER")

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
    saCurrentDelay = q_config['time'] #delay.get()
    global saCallback
    global saQuestionIndex
    saQuestionIndex = q_config['index'] - 1
    saCallback = threading.Timer(saCurrentDelay, askSA, [saQuestionIndex, saQuestionIndex, saQuestionIndex, arena_config])
    saCallback.start()
    global saStartTime
    saStartTime = datetime.now()

    #delaySelect.pack_forget()
    #sa1Select.pack_forget()
    #sa2Select.pack_forget()
    #sa3Select.pack_forget()

def pauseSATimer():
    global saCallback
    if(saCallback):
        saCallback.cancel()
        saCallback = None
    totalTimeRun = pauseStartTime - saStartTime
    global saCurrentDelay
    saCurrentDelay -= totalTimeRun.total_seconds()

def unPauseSATimer():
    global saCallback
    if saCallback:
        saCallback.cancel()
        saCallback = None
    saCallback = threading.Timer(saCurrentDelay, askSA, [saQuestionIndex, saQuestionIndex, saQuestionIndex, arena_config])
    saCallback.start()
    global saStartTime
    saStartTime = datetime.now()

def clearSATimer():
    global isSAStarted
    isSAStarted = False
    checkpointBtn.configure(text="CHECKPOINT REACHED")
    global saCurrentDelay
    saCurrentDelay = -1
    global saCallback
    if(saCallback):
        saCallback.cancel()
        saCallback = None
    global saStartTime
    saStartTime = -1

    saTimeLabel.config(text='')

#    delaySelect.pack()
#    sa1Select.pack()
#    sa2Select.pack()
#    sa3Select.pack()

def resetToCheckpoint():
    global isReset
    if isReset:
        return

    if not isStarted or not isPaused:
        resetLabel.configure(text="Run must be paused to reset!")
        return

    if isSAStarted:
        clearSATimer()
        global shouldRestartSA
        shouldRestartSA = True

    isReset = True

    global activeCheckPointResetDuration
    timeReset = (pauseStartTime - continueTime) - activeCheckPointPauseDuration - activeCheckPointResetDuration
    activeCheckPointResetDuration += timeReset
    global totalResetDuration
    totalResetDuration += timeReset

def addCheckpoint(duration):
    global activeCheckPointPauseDuration
    activeCheckPointPauseDuration = timedelta(0)
    global activeCheckPointResetDuration
    activeCheckPointResetDuration = timedelta(0)

    checkpoints.append(duration)

    currentPauseElapsedTime = timedelta(0)
    if isPaused:
        currentPauseElapsedTime = datetime.now() - pauseStartTime
    checkpoint_no_pause = duration - currentPauseElapsedTime - totalPauseDuration - totalResetDuration

    txt = timeSplitLabel['text'] + '\n' + str(checkpoint_no_pause).split(".")[0]
    timeSplitLabel.configure(text=txt)

def askSA(q1, q2, q3, arena): # keeping 3 arguments so we can change back easily if needed
    global currentSAIndex
    rospy.set_param('/user_study/participant_id', id)
    rospy.set_param('/user_study/run_number', currentRunID)
    rospy.set_param('/user_study/interrupt_number', currentSAIndex + 1)

    questionStr = SA1_OPTIONS[q1] + ";" + SA2_OPTIONS[q2] + ";" + SA3_OPTIONS[q3]  #sa1.get() + ";" + sa2.get() + ";" + sa3.get()

    clearSATimer()

    currentSAIndex = currentSAIndex + 1

    obstacle_config = OBSTACLE_COLORS[currentRunID].split(";")

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
    global checkpointBtn
    checkpointBtn = tk.Button(master=frame, text="CHECKPOINT REACHED", width=30,
                      command=onCheckpointReached)
    checkpointBtn.pack(padx=5, pady=5)
    global saTimeLabel
    saTimeLabel = tk.Label(master=frame, text='')
    saTimeLabel.pack(padx=5, pady=5)

    frame = tk.Frame(
        master=window,
        relief=tk.RAISED,
        borderwidth=1
    )

    global delay
    delay = tk.IntVar(frame)
    delay.set(DELAY_OPTIONS[0])

    global sa1
    sa1 = tk.StringVar(frame)
    sa1.set(SA1_OPTIONS[0])

    global sa2
    sa2 = tk.StringVar(frame)
    sa2.set(SA2_OPTIONS[0])

    global sa3
    sa3 = tk.StringVar(frame)
    sa3.set(SA3_OPTIONS[0])

    frame.grid(row=1, column=2, padx=5, pady=5)

    global delaySelect
    delaySelect = tk.OptionMenu(frame, delay, *DELAY_OPTIONS)
#    delaySelect.pack()

    global sa1Select
    sa1Select = tk.OptionMenu(frame, sa1, *SA1_OPTIONS)
#    sa1Select.pack()
    global sa2Select
    sa2Select = tk.OptionMenu(frame, sa2, *SA2_OPTIONS)
#    sa2Select.pack()
    global sa3Select
    sa3Select = tk.OptionMenu(frame, sa3, *SA3_OPTIONS)
#    sa3Select.pack()

    global resetBtn
    resetBtn = tk.Button(master=frame, text="RESET TO CHECKPOINT", width=30,
                      command=resetToCheckpoint)
    resetBtn.pack(padx=5, pady=5)
    global resetLabel
    resetLabel = tk.Label(master=frame, text='')
    resetLabel.pack(padx=5, pady=5)
    global timeSplitLabel
    timeSplitLabel = tk.Label(master=frame, text='')
    timeSplitLabel.pack(padx=5, pady=5)


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
