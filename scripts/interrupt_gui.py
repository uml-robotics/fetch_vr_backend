#!/usr/bin/env python

import Tkinter as tk
from Tkinter import *
import PIL
from PIL import ImageTk, Image
#import ImageTK
#import Image
import rospy
from std_msgs.msg import String
import rospkg

class InterruptQuestions:

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

    MC_ANSWERS = { 
        "What color is the obstacle closest to the robot?" : "Yellow;Orange;Grey;None;Don't know",
        "How many objectives has the robot reached thus far?": "0;1;2;None;Don't know",
        "Out of the three objectives, how many are remaining?":"3;2;1;None;Don't know",
        "What color is the obstacle closest to the robot?": "Yellow;Orange;Grey;None;Don't know",
    }

    # Uses a map image in the prompt
    MC_MAP_ANSWERS = { 
        "Based on where the robot is currently located on the grid, if the robot drives up 2 grid blocks, will it run into any obstacles?" : "Yes;No;Don't know"
    }

    SLIDER_ANSWERS = { 
        "In terms of distance, what percent of the way are you to your next target? Please use the slider." : "Submit"
    }

    GRID_ANSWERS = {
        "Where is the obstacle closest to the robot?": "None;Don't know",
        "In which direction would the robot need to turn toward to be at the next target?": "None;Don't know"}


    MAP_ANSWERS = {
        "Where is the robot on this map?": "None;Don't know",
        "Where was the last objective the robot reached? If you have not yet reached an objective, where was the starting location?": "None;Don't know",
        "Where is the next objective?": "None;Don't know"
        }


    current_question = 0

    def create_window(self):
        self.window = tk.Tk()
        self.window.geometry("1920x12080")
        #self.window.overrideredirect(1)
        self.window.withdraw()
        #self.window.wm_attributes("-topmost", 1)

        #self.window.attributes("-fullscreen", True)
        #self.window.attributes("-minimize", True)

        for i in range(3):
            self.window.columnconfigure(i, weight=1, minsize=100)
            self.window.rowconfigure(i, weight=1, minsize=100)

        #self.window.focus_force()

    def destroy_window(self):
        #self.window.destroy()
        self.window.state(newstate='iconic')

    def show_window(self):
        self.window.state(newstate='normal')

    def __init__(self):
        self.sub = rospy.Subscriber("/question", String, self.callback)
        self.create_window()
        self.window.withdraw()
        #self.destroy_window()
        self.rospack = rospkg.RosPack()
        self.window.mainloop()



    def setupMultipleChoiceQuestion(self, question):
        print("Setting up multiple choice question")

        #self.create_window()
        self.frame = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )
        self.frame.grid(row=0, column=0, padx=1, pady=1)
        #self.frame.pack()
        print("Setting up multiple choice question2")

        timeLabel = tk.Label(master=self.frame, text=question)
        timeLabel.pack(padx=5, pady=5)

        if self.MC_ANSWERS.has_key(question):
            print("HAS KEY")
        
        answers = self.MC_ANSWERS[question].split(';')
        print("Setting up multiple choice question3")
        for answer in answers:
            Choice = tk.Button(master=self.frame, text=answer, width=30,command=self.onButtonPressed)
            Choice.pack()


    def setupGridQuestion(self, question):
        print("setting up grid question")
        self.gridframe = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )

        self.gridframe.grid(row=0, column=0, padx=1, pady=1)

        timeLabel = tk.Label(master=self.gridframe, text=question)
        timeLabel.pack(padx=5, pady=5)

        self.img = tk.PhotoImage(file=(self.rospack.get_path('fetch_vr_backend')+"/resources/grid_relative_no_lines.png"))

        self.canvas = tk.Canvas(self.gridframe, width=640, height=480)
        self.canvas.pack()
        self.canvas.create_image(self.canvas.winfo_width() / 2,self.canvas.winfo_height() / 2, anchor=CENTER, image=self.img)

        if self.GRID_ANSWERS.has_key(question):
            print("HAS KEY")
        
        answers = self.GRID_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.gridframe, text=answer, width=30,command=self.onButtonPressed)
            Choice.pack()

#On grid clicked add buttons


    def setupMapQuestion(self,question):
        print("setting up map question")
        self.mapframe = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )

        timeLabel = tk.Label(master=self.mapframe, text=question)
        timeLabel.pack(padx=5, pady=5)

        self.mapframe.grid(row=0, column=0, padx=1, pady=1)

        self.img = tk.PhotoImage(file=(self.rospack.get_path('fetch_vr_backend')+"/resources/GridMap.png"))

        self.canvas = tk.Canvas(self.mapframe, width=640, height=480)
        self.canvas.pack()
        self.canvas.create_image(self.canvas.winfo_width() / 2,self.canvas.winfo_height() / 2, anchor=CENTER, image=self.img)

        if self.MAP_ANSWERS.has_key(question):
            print("HAS KEY")
        
        answers = self.MAP_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.mapframe, text=answer, width=30,command=self.onButtonPressed)
            Choice.pack()

#On map clicked add buttons


    def process_next_question(self):
        print(self.question[self.current_question])


        if hasattr(self, 'frame'):
            self.frame.destroy()
        if hasattr(self, 'mapframe'):
            self.mapframe.destroy()
        if hasattr(self, 'gridframe'):
            self.gridframe.destroy()

        if self.current_question < 2:
            if self.MC_ANSWERS.has_key(self.question[self.current_question]):
                self.setupMultipleChoiceQuestion(self.question[self.current_question])
            elif self.MAP_ANSWERS.has_key(self.question[self.current_question]):
                print ("MAP NOT IMPLEMENTED YET")
                self.setupMapQuestion(self.question[self.current_question])
            elif self.GRID_ANSWERS.has_key(self.question[self.current_question]):
                print ("Grid NOT IMPLEMENTED YET")
                self.setupGridQuestion(self.question[self.current_question])
            else:
                print( "QUESTION NOT RECOGNIZED??")
                self.window.withdraw()
        else:
            self.window.withdraw()

    def onButtonPressed(self):
        self.current_question = self.current_question + 1
        self.process_next_question()



    def callback(self, msg):
        self.question = msg.data.split(';')
        self.current_question = 0
        self.window.deiconify()
        self.process_next_question()



if __name__ == '__main__':

    rospy.init_node("interfaceinterruptquestions")
    InterruptQuestions()
    rospy.spin()
    


#for i in range(3):
#    window.columnconfigure(i, weight=1, minsize=1920)
#    window.rowconfigure(i, weight=1, minsize=1280)

#frame = tk.Frame(
#    master=window,
#    relief=tk.RAISED,
#    borderwidth=1
#)



#window.focus_force()
#window.withdraw()

