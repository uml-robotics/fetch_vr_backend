#!/usr/bin/env python

from matplotlib import offsetbox
import Tkinter as tk
from Tkinter import *
import PIL
from PIL import ImageTk, Image
#import ImageTK
#import Image
import rospy
from std_msgs.msg import String
import rospkg
import os

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
        "Based on where the robot is currently located on the grid, if the robot drives up 2 grid blocks, will it run into any obstacles?" : "Yes;No;Don't know",
        "Based on where the robot is currently located on the grid, if the robot drives down 2 grid blocks, will it run into any obstacles." : "Yes;No;Don't know",
        "Based on where the robot is currently located on the grid, if the robot drives right 2 grid blocks, will it run into any obstacles." : "Yes;No;Don't know"
    }

    SLIDER_ANSWERS = { 
        "In terms of distance, what percent of the way are you to your next target? Please use the slider." : "Don't know"
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
        self.answerPub = rospy.Publisher('answer', String, queue_size=10)
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

        timeLabel = tk.Label(master=self.frame, text=question)
        timeLabel.pack(padx=5, pady=5)
        
        answers = self.MC_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.frame, text=answer, width=30,command=lambda current_answer=answer: self.onButtonPressed(current_answer))
            Choice.pack()


    def setupGridQuestion(self, question):
        print("setting up grid question")
        self.gridframe = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )

        self.marker = []
        self.marker_location = None

        self.gridframe.grid(row=0, column=0, padx=0, pady=0)

        timeLabel = tk.Label(master=self.gridframe, text=question)
        timeLabel.pack(padx=5, pady=5)

        self.img = tk.PhotoImage(file=(self.rospack.get_path('fetch_vr_backend')+"/resources/grid_relative_no_lines.png"))

        self.canvas = tk.Canvas(self.gridframe, width=self.img.width(), height=self.img.height())
        self.canvas.pack()
        self.canvas.create_image(self.img.width() / 2,self.img.height() / 2, anchor=CENTER, image=self.img)
        self.draw_grid(3)
        self.canvas.bind("<Button 1>", self.on_grid_clicked)
        
        answers = self.GRID_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.gridframe, text=answer, width=30,command=lambda current_answer=answer: self.onButtonPressed(current_answer))
            Choice.pack()


    def setupMapQuestion(self,question):
        print("setting up map question")
        self.mapframe = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )

        self.marker = []
        self.marker_location = None

        timeLabel = tk.Label(master=self.mapframe, text=question)
        timeLabel.pack(padx=5, pady=5)

        self.mapframe.grid(row=0, column=0, padx=0, pady=0)

        self.img = tk.PhotoImage(file=(self.rospack.get_path('fetch_vr_backend')+"/resources/GridMap.png"))

        self.canvas = tk.Canvas(self.mapframe, width=self.img.width(), height=self.img.height())
        self.canvas.pack()
        self.canvas.create_image(self.img.width() / 2,self.img.height() / 2, anchor=CENTER, image=self.img)
        self.canvas.bind("<Button 1>", self.on_map_clicked)
        
        answers = self.MAP_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.mapframe, text=answer, width=30,command=lambda current_answer=answer: self.onButtonPressed(current_answer))
            Choice.pack()


    def setupMCMapQuestion(self,question):
        print("setting up mc map question")
        self.mapframe = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )

        timeLabel = tk.Label(master=self.mapframe, text=question)
        timeLabel.pack(padx=5, pady=5)

        self.mapframe.grid(row=0, column=0, padx=0, pady=0)

        self.img = tk.PhotoImage(file=(self.rospack.get_path('fetch_vr_backend')+"/resources/GridMap.png"))

        self.canvas = tk.Canvas(self.mapframe, width=self.img.width(), height=self.img.height())
        self.canvas.pack()
        self.canvas.create_image(self.img.width() / 2,self.img.height() / 2, anchor=CENTER, image=self.img)
        
        self.draw_grid(10)

        answers = self.MC_MAP_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.mapframe, text=answer, width=30,command=lambda current_answer=answer: self.onButtonPressed(current_answer))
            Choice.pack()

    def setupSliderQuestion(self, question):
        print("Setting up slider question")

        #self.create_window()
        self.frame = tk.Frame(
            master=self.window,
            relief=tk.RAISED,
            borderwidth=1
        )
        self.frame.grid(row=0, column=0, padx=1, pady=1)

        timeLabel = tk.Label(master=self.frame, text=question)
        timeLabel.pack(padx=5, pady=5)

        self.slider_value = tk.IntVar()

        slider = tk.Scale(master=self.frame, variable=self.slider_value, from_=0, to=100, length=200, orient=tk.HORIZONTAL)  
        slider.pack()

        answers = self.SLIDER_ANSWERS[question].split(';')
        for answer in answers:
            Choice = tk.Button(master=self.frame, text=answer, width=30,command=lambda current_answer=answer: self.onButtonPressed(current_answer))
            Choice.pack()
        submit_button = tk.Button(master=self.frame, text="Submit", width=30,command=lambda:self.onButtonPressed(self.slider_value.get()))
        submit_button.pack()


    def process_next_question(self):
        print(self.question[self.current_question])

        if hasattr(self, 'frame'):
            self.frame.destroy()
        if hasattr(self, 'mapframe'):
            self.mapframe.destroy()
        if hasattr(self, 'gridframe'):
            self.gridframe.destroy()

        if self.MC_ANSWERS.has_key(self.question[self.current_question]):
            self.question_type = "mc"
            self.setupMultipleChoiceQuestion(self.question[self.current_question])
        elif self.MAP_ANSWERS.has_key(self.question[self.current_question]):
            self.question_type = "map"
            self.setupMapQuestion(self.question[self.current_question])
        elif self.GRID_ANSWERS.has_key(self.question[self.current_question]):
            self.question_type = "grid"
            self.setupGridQuestion(self.question[self.current_question])
        elif self.MC_MAP_ANSWERS.has_key(self.question[self.current_question]):
            self.question_type = "mc"
            self.setupMCMapQuestion(self.question[self.current_question])
        elif self.SLIDER_ANSWERS.has_key(self.question[self.current_question]):
            self.question_type = "slider"
            self.setupSliderQuestion(self.question[self.current_question])
        else:
            print( "QUESTION NOT RECOGNIZED??")
            self.window.withdraw()

    def onButtonPressed(self, text):
        participant_id = '-1'
        if rospy.has_param('/user_study/participant_id'):
            participant_id = rospy.get_param('/user_study/participant_id')

        run_number = -1
        if rospy.has_param('/user_study/run_number'):
            run_number = rospy.get_param('/user_study/run_number')

        interrupt_number = -1
        if rospy.has_param('/user_study/interrupt_number'):
            interrupt_number = rospy.get_param('/user_study/interrupt_number')

        level = -1
        if self.question[self.current_question] in self.SA1_OPTIONS:
            level = 1
        elif self.question[self.current_question] in self.SA2_OPTIONS:
            level = 2
        elif self.question[self.current_question] in self.SA3_OPTIONS:
            level = 3


        response_str = str(participant_id) + ";2D;" + str(run_number) + ";" + str(interrupt_number) + ";" + str(level) + ";" + self.question_type + ";" +self.question[self.current_question] + ";" + str(text)
        print(response_str)
        self.answerPub.publish(response_str)

        data_path = self.rospack.get_path('fetch_vr_backend')+"/data"
        if not os.path.exists(data_path):
            os.makedirs(data_path)
        full_file_path = data_path + "/Participant_" + str(participant_id) + ".csv"
        if not os.path.isfile(full_file_path):
            with open(full_file_path, 'w') as f:
                f.write("ParticipantID;Interface;Run;Interruption;SALevel;Type;Question;Answer\n")
        with open(full_file_path, 'a') as f:
            f.write(response_str + '\n')

        self.current_question = self.current_question + 1
        if self.current_question < 3:
            self.process_next_question()
        else:
            self.window.withdraw()

    def callback(self, msg):
        self.question = msg.data.split(';')
        self.current_question = 0
        self.window.deiconify()
        self.process_next_question()


    def on_grid_clicked(self, event):
        '''Callback for when the user clicks the mouse'''
        x_coord = (event.x // self.grid_size) * self.grid_size + self.grid_size / 2 + 1
        y_coord = (event.y // self.grid_size) * self.grid_size + self.grid_size / 2 + 1

        self.marker_location = ((event.x // self.grid_size), (event.y // self.grid_size))

        # MIDDLE TILE PRESSED
        if event.x // self.grid_size == 1 and event.y // self.grid_size == 1:
            self.reset_marker()
            return

        if len(self.marker) != 4:
            self.marker.append(self.canvas.create_line((x_coord - self.grid_size / 2, y_coord + self.grid_size / 2,
                                         x_coord + self.grid_size / 2, y_coord + self.grid_size / 2), fill='red'))

            self.marker.append(self.canvas.create_line((x_coord - self.grid_size / 2, y_coord - self.grid_size / 2,
                                         x_coord + self.grid_size / 2, y_coord - self.grid_size / 2), fill='red'))

            self.marker.append(self.canvas.create_line((x_coord + self.grid_size / 2, y_coord - self.grid_size / 2,
                                         x_coord + self.grid_size / 2, y_coord + self.grid_size / 2), fill='red'))

            self.marker.append(self.canvas.create_line((x_coord - self.grid_size / 2, y_coord - self.grid_size / 2,
                                         x_coord - self.grid_size / 2, y_coord + self.grid_size / 2), fill='red'))
            self.on_marker_created(self.gridframe)
        else:
            self.canvas.coords(self.marker[0], (x_coord - self.grid_size / 2, y_coord + self.grid_size / 2,
                                         x_coord + self.grid_size / 2, y_coord + self.grid_size / 2))
            self.canvas.coords(self.marker[1], (x_coord - self.grid_size / 2, y_coord - self.grid_size / 2,
                                         x_coord + self.grid_size / 2, y_coord - self.grid_size / 2))
            self.canvas.coords(self.marker[2], (x_coord + self.grid_size / 2, y_coord - self.grid_size / 2,
                                         x_coord + self.grid_size / 2, y_coord + self.grid_size / 2))
            self.canvas.coords(self.marker[3], (x_coord - self.grid_size / 2, y_coord - self.grid_size / 2,
                                         x_coord - self.grid_size / 2, y_coord + self.grid_size / 2))


    def on_map_clicked(self, event):
        '''Callback for when the user clicks the mouse'''
        x_coord = event.x    
        y_coord = event.y

        self.marker_location = (x_coord,y_coord)

        if len(self.marker) != 2:
            self.marker.append(self.canvas.create_line((x_coord + 10, y_coord + 10,
                                         x_coord - 10, y_coord - 10), fill='red'))
            self.marker.append(self.canvas.create_line((x_coord - 10, y_coord + 10,
                                         x_coord + 10, y_coord - 10), fill='red'))
            self.on_marker_created(self.mapframe)
        else:
            self.canvas.coords(self.marker[0], (x_coord + 10, y_coord + 10,
                                         x_coord - 10, y_coord - 10))
            self.canvas.coords(self.marker[1], (x_coord - 10, y_coord + 10,
                                         x_coord + 10, y_coord - 10))

    def draw_grid(self, num_rows):
        self.grid_size = self.img.width() / num_rows
        for i in range(1, int(self.img.width() + 1), self.grid_size):
            self.canvas.create_line((i, 0,
                                     i, self.img.height()), fill='black')
        for i in range(1, int(self.img.height() + 1), self.grid_size):
            self.canvas.create_line((0, i,
                                     self.img.width(), i), fill='black')
                                     
    def on_marker_created(self, frame):
        self.submit_button = tk.Button(master=frame, text="Submit", width=30,command=lambda:self.onButtonPressed(self.marker_location))
        self.submit_button.pack()
        self.reset_button = tk.Button(master=frame, text="Reset", width=30,command=self.reset_marker)
        self.reset_button.pack()

    def reset_marker(self):
        for component in self.marker:
            self.canvas.delete(component)
        self.marker = []
        if hasattr(self, 'submit_button'):
            self.submit_button.pack_forget()
        if hasattr(self, 'reset_button'):
            self.reset_button.pack_forget()

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

