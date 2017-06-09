#!/usr/bin/python
"""Author-Temaldegn_Yisfa"""
try:
    import Tkinter as tk
    import ttk
except:
    import tkinter as tk
    from tkinter import ttk

import tkFileDialog
import EmotiveSpeech as emos
import synthesis as syn
import math
import rospy
import os

# rospy.init_node('emotrans')
class EmoGUI:
    def __init__(self):
        self.master=tk.Tk()
        self.master.title("Emotion")
        self.master.style=ttk.Style()
        self.master.style.theme_use("clam")
        #('clam', 'alt', 'default', 'classic')
        # self.master.style=ttk.Style().configure("TButton", padding=6, relief="flat", background="#ccc")
        self.master.geometry('{}x{}'.format(570, 350))
        self.master.resizable(width=False, height=False)
    
        self.file_path=tk.StringVar()
        self.files=[]
        self.selected_emotion=tk.StringVar()
        self.sampling_frequency=tk.IntVar()
        self.emotion_intensity=tk.DoubleVar()
        self.log_chunk=tk.IntVar()
        self.chunk_size=tk.IntVar()
        self.message_to_user=tk.StringVar()


#File browser       
#*************************************************#
        self.browse_frame=tk.Frame(self.master, height=20, width=300)
        self.browse_frame.pack()
        self.browse_frame.place(x=60, y=70)
        
        
        self.path_field=tk.Entry(self.browse_frame, textvariable=self.file_path, width=45)
        self.path_field.pack(side=tk.LEFT)
        
        self.browse_file_button=tk.Button(self.browse_frame, text='Choose File/s', command=self.open_file)
        self.browse_file_button.pack(side=tk.RIGHT)

        
#Emotion and intensity
#*************************************************#
        self.emotion_frame=tk.Frame(self.master, height=300, width=200)
        self.emotion_frame.pack()
        self.emotion_frame.place(x=60, y=110)

        self.emo_label=tk.Label(self.emotion_frame, text="\nGenerate file for")
        self.emo_label.pack()
        
        self.radio_emotion=[]
        radio_list=['Happy', 'Sad', 'Afraid', 'Happy Tensed']
        emo_list=['happy', 'sad', 'afraid', 'happy_tensed']

        for i, rl in enumerate(radio_list):
            rbtn=tk.Radiobutton(self.emotion_frame, text=rl, variable=self.selected_emotion, value=emo_list[i], command=self.update_emotion_param)
            self.radio_emotion.append(rbtn)
            self.radio_emotion[i].pack(anchor=tk.W)


        self.int_label=tk.Label(self.emotion_frame, text="\nEmotion Intensity")
        self.int_label.pack()
        
        self.emo_int=tk.Scale(self.emotion_frame, variable=self.emotion_intensity,orient=tk.HORIZONTAL, length=100, command=self.update_intensity, showvalue=0)
        self.emo_int.pack()
        self.emo_int.set(100)
        
        self.percent=tk.Label(self.emotion_frame, text=str(self.emo_int.get())+'%')
        self.percent.pack(anchor=tk.E)
        
        self.radio_emotion[0].select()
      
#Sampling frequency
#*************************************************#
        self.frequency_frame=tk.Frame(self.master, height=50, width=50)
        self.frequency_frame.pack()
        self.frequency_frame.place(x=340, y=110)
        
        self.freq_label=tk.Label(self.frequency_frame, text="\nSampling Frequency")
        self.freq_label.pack()
        
        frequecy_list=['8,000 Hz','11,025 Hz','22,050 Hz','44,100 Hz']

        self.fs_list=ttk.Combobox(self.frequency_frame,state='readonly',  values=frequecy_list)
        self.fs_list.current(0)
        self.sampling_frequency.set(8000)
        self.fs_list.pack()
        self.fs_list.bind("<<ComboboxSelected>>", self.update_frequency)
        
#Chunk size
#*************************************************#
        self.chunk_frame=tk.Frame(self.master, width=200, height=200)
        self.chunk_frame.pack()
        self.chunk_frame.place(x=340, y=200)
        
        self.chunk_label=tk.Label(self.chunk_frame, text="\nChunk Size")
        self.chunk_label.pack(anchor=tk.W)

        self.two_label=tk.Label(self.chunk_frame, text ="2 ^ ")
        self.two_label.pack(side=tk.LEFT)

        self.log_chunk.set(10)
        self.chunk_size.set(1024)

        self.chunk_spinner=tk.Spinbox(self.chunk_frame,state="readonly",width=15, textvariable=self.log_chunk, from_=6, to=20, wrap=True, command=self.update_chunk)
        self.chunk_spinner.pack(side=tk.RIGHT)

        self.size_label=tk.Label(self.chunk_frame, textvariable=self.chunk_size)
        self.size_label.pack()
        self.size_label.place(x=100, y=15)


###############
#*************************************************#

        self.result_frame=tk.Frame(self.master, width=260, height=70)
        self.result_frame.pack()
        self.result_frame.place(x=300, y=280)

        self.generate_button=tk.Button(self.result_frame,text='Generate', command=self.generate_file)
        self.generate_button.pack()
        self.generate_button.place(x=140, y=0)

        self.message_label=tk.Label(self.result_frame, textvariable=self.message_to_user)
        self.message_label.pack()
        self.message_label.place(x=30, y=30)

        self.load_parameters()

#*************************************************#
        #********** END OF WIDGETS **********#

    
       

    def update_emotion_param(self):
        self.set_ros_param('selected_emotion', self.selected_emotion.get())

    def update_frequency(self, event):
        freq=self.fs_list.get()
        s=''
        for c in freq[0:len(freq)-3]:
            if c!=',':
                s+=c
        self.sampling_frequency.set(int(s))    
        
        self.set_ros_param('sampling_frequency', self.sampling_frequency.get())

    def update_intensity(self,val):
        self.percent.config(text=str(val)+'%')
        
        self.set_ros_param('~emotion_intensity', float(val)/100.0)

    def open_file(self):
        del self.files[:]
        self.file_path.set(tkFileDialog.askopenfilenames(initialdir='.', title="Select File", filetypes=(("WAVE files","*.wav"), ("All files", "*.*"))))

        for index, element in enumerate(self.file_path.get().split()):
            self.files.append(element[(1/(index+1)+1):(-2-1/len(self.file_path.get().split()))])

        
        self.set_ros_param('file_names', self.files)


    def update_chunk(self):
        self.chunk_size.set(pow(2, self.log_chunk.get()))
        
        self.set_ros_param('chunk_size', self.chunk_size.get())

    def generate_file(self):
        # saving_directory=tkFileDialog.asksaveasfilename()
        # print saving_directory

        # self.set_ros_param('file_path', self.path_field.get())
        self.load_parameters()
        
        # try:
        #     emos.generate_file(self.get_ros_param('selected_emotion'))
        # except:
        #     if not self.file_path.get()=="":
        #       self.set_error_message(1)
        #     else:
        #       self.set_error_message(2)
        emos.generate_file(self.get_ros_param('selected_emotion'))

            

    def load_parameters(self):
        if self.check_ros_param('~selected_emotion'):
            self.selected_emotion.set(self.get_ros_param('~selected_emotion'))
        else:
            self.set_ros_param('~selected_emotion', self.selected_emotion.get())
        if self.check_ros_param('~emotion_intensity'):
            self.emotion_intensity.set(self.get_ros_param('~emotion_intensity')*100.0)
            self.percent.config(text=str(int(self.emotion_intensity.get()))+'%')
            syn.PARAMETER_CONTROL=self.emotion_intensity.get()/100.0
        else:
            self.set_ros_param('~emotion_intensity',self.emotion_intensity.get())
        if self.check_ros_param('~chunk_size'):
            self.chunk_size.set(self.get_ros_param('~chunk_size'))
            self.log_chunk.set(int(math.log(self.chunk_size.get())/math.log(2)))
            emos.CHUNK_SIZE=self.chunk_size.get()
        else:
            self.set_ros_param('~chunk_size', self.chunk_size.get())
        if self.check_ros_param('~sampling_frequency'):
            self.sampling_frequency.set(self.get_ros_param('~sampling_frequency'))
            self.fs_list.current((self.sampling_frequency.get()+2)/11026)
        else:
            self.set_ros_param('~sampling_frequency', self.sampling_frequency.get())
        if self.check_ros_param('~file_names'):
            self.files=(self.get_ros_param('file_names'))
        else:
            self.set_ros_param('~file_names', self.files)

    def set_error_message(self, err_cod):
        error_message_list=['', 
        'No such file, please enter a valid path \n try using browser', 
        'Please enter file path', 
        'Unable to communicate with master \n try starting roscore']
        self.message_to_user.set(error_message_list[err_cod])

    def set_ros_param(self, name, value):
        rospy.init_node('emotrans')
        try:
            rospy.set_param(name, value)
        except:
            self.set_error_message(3)
    def get_ros_param(self, name):
        try:
            return rospy.get_param(name)
        except:
            self.set_error_message(3)
    def check_ros_param(self, name):
        try:
            return rospy.has_param(name)
        except:
            self.set_error_message(3)


my_gui=EmoGUI()
tk.mainloop()




#key typed event for:
    #setting rosparam /file_path while typing


