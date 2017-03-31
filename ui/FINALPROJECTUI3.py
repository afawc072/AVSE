from tkinter import*
import os,time
import subprocess
def start(frame):
    frame.tkraise()
root = Tk()
root.geometry("320x240")
f1 = Frame(root)
f2= Frame(root)
f3 = Frame(root)
f4 = Frame(root)
f5=Frame(root)
f6=Frame(root)
f7=Frame(root)
f8=Frame(root)
f9=Frame(root)
f10=Frame(root)
f11=Frame(root)
f12=Frame(root)
#define event where a button triggers more than one command
def sequence(*functions):
    def func(*args, **kwargs):
        return_value = None
        for function in functions:
            return_value = function(*args, **kwargs)
        return return_value
    return func
# set up the commands for buttons
def kitchen():
    subprocess.call(["./bin/AVSE","0"])
def bathroom():
     subprocess.call(["./bin/AVSE","1"])
def bedroom():
     subprocess.call(["./bin/AVSE","2"])
def game():
     subprocess.call(["./bin/AVSE","3"])
def loft():
     subprocess.call(["./bin/AVSE","4"])
def laundry():
     subprocess.call(["./bin/AVSE","5"])
def livingroom():
     subprocess.call(["./bin/AVSE","6"])
def pantry():
     subprocess.call(["./bin/AVSE","7"])

def arrived():
    start (f11)
def exit():
    start(f12)
    print ("exit authourized")
def cont():
    filesizestored=os.stat('Dest.txt').st_size
    while True:
        try:
            currentfilesize=os.stat('Dest.txt').st_size
            if filesizestored != currentfilesize:
                f=open("Dest.txt")
                if f.read()=="finish":
                   start(f11)
                   break
                
        except:
            pass
   

# Making the windows and setting parameters like screen size and relating all widgets to a window 


Frame.configure(f1, background='slate gray2', relief='sunken')
Frame.configure(f2, background='slate gray2')
Frame.configure(f3, background='slate gray2')
Frame.configure(f4, background='slate gray2')
Frame.configure(f5, background='slate gray2')
Frame.configure(f6, background='slate gray2')
Frame.configure(f7, background='slate gray2')
Frame.configure(f8, background='slate gray2')
Frame.configure(f9, background='slate gray2')
Frame.configure(f10, background='slate gray2')
Frame.configure(f11, background='slate gray2')
Frame.configure(f12, background='red')


for frame in (f1, f2, f3, f4,f5,f6,f7,f8,f9,f10,f11,f12):
    frame.grid(row=0, column=0, sticky='news')
    

#initalize and interconnect screens
Label(f1,pady=6,text='WELCOME TO THE INDOOR VEHICLE\n to begin please press the start button',bg='slate gray2',font=('MS serif',12,'bold')).pack()
startbutton=Button(f1,padx=8,pady=10,text='START',font=('MS serif',14,'bold'),bg="gold",command=lambda:start(f2)).pack()
       
menu1button=Button(f2,padx=2,pady=3,text='KITCHEN',bg="gold",relief= 'raised',font=('MS serif',12,'bold'),command=lambda:start(f3)).grid(row=0, column=1,padx=6, pady=5)
                  
menu2button=Button(f2,pady=3,text='BATHROOM',font=('MS serif',12,'bold'),bg="gold",relief= 'raised',command=lambda:start(f4)).grid(row=0, column=2 ,padx=6, pady=5)

menu3button=Button(f2,pady=3,text='LIVINGROOM',font=('MS serif',12,'bold'),relief= 'raised',bg="gold",command=lambda:start(f5)).grid(row=0, column=3,padx=6, pady=5)

menu4button=Button(f2,pady=3,text='BEDROOM',font=('MS serif',12,'bold'),relief= 'raised',bg="gold",command=lambda:start(f7)).grid(row=1, column=1,padx=6, pady=5)

menu5button=Button(f2,pady=3,text='GAME ROOM',font=('MS serif',12,'bold'),bg="gold",command=lambda:start(f8)).grid(row=1, column=2,padx=6, pady=5)

menu6button=Button(f2,padx=20,pady=3,text='LOFT',font=('MS serif',12,'bold'),bg="gold",command=lambda:start(f9)).grid(row=1, column=3,padx=6, pady=5)

menu7button=Button(f2,padx=8,pady=3,text='PANTRY',font=('MS serif',12,'bold'),bg="gold",command=lambda:start(f10)).grid(row=2, column=1,padx=6, pady=5)

menu8button=Button(f2,padx=10,pady=3,text='LAUNDRY',font=('MS serif',12,'bold'),bg="gold",command=lambda:start(f6)).grid(row=2, column=2,padx=6, pady=5)

menu9button=Button(f2,padx=22,pady=3,text='EXIT',font=('MS serif',12,'bold'),bg="gold",command=exit).grid(row=2, column=3,padx=6, pady=5)

Label(f3,bg='slate gray2',text='YOUR DESTINATION IS THE KITCHEN\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f3,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f3,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(kitchen, cont)).pack(side='bottom')

Label(f4,bg='slate gray2',text='YOUR DESTINATION IS THE BATHROOM\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f4,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f4,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=bathroom).pack(side='bottom')

Label(f5,bg='slate gray2',text='YOUR DESTINATION IS THE LIVING ROOM\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f5,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f5,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(livingroom, cont)).pack(side='bottom')

Label(f6,bg='slate gray2',text='YOUR DESTINATION IS THE LAUNDRY\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f6,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f6,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(laundry,cont)).pack(side='bottom')

Label(f7,bg='slate gray2',text='YOUR DESTINATION IS THE BEDROOM\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f7,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f7,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(bedroom,cont)).pack(side='bottom')

Label(f8,bg='slate gray2',text='YOUR DESTINATION IS THE GAME ROOM\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f8,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f8,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(game,cont)).pack(side='bottom')

Label(f9,bg='slate gray2',text='YOUR DESTINATION IS THE LOFT\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f9,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f2)).pack(side='bottom')
ct=Button(f9,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(loft,cont)).pack(side='bottom')

Label(f10,bg='slate gray2',text='YOUR DESTINATION IS THE PANTRY\n to change destination please press the back button,\n to continue please press the continue button',font=('MS serif',12,'bold')).pack()
BIT= Button(f10,text='BACK',font=('MS serif',12,'bold'),bg='brown1',command=lambda:start(f11)).pack(side='bottom')
ct=Button(f10,bg='gold',text='CONTINUE',font=('MS serif',12,'bold'),command=sequence(pantry,cont)).pack(side='bottom')

Label(f11,bg='slate gray2',text='YOU HAVE ARRIVED AT YOUR DESTINATION',font=('MS serif',12,'bold')).pack()
bit=Button(f11,text='EXIT',font=('MS serif',12,'bold'),bg='pink3',command=lambda:start(f1)).pack(side='bottom')
CT1=Button(f11,bg='gold',text='MAKE A NEW SELECTION',font=('MS serif',12,'bold'),command=lambda:start(f2)).pack(side='bottom')

Label(f12,bg='red',fg='white',text='DESTINATION NOT FOUND',font=('MS serif',16,'bold')).pack()
bit=Button(f12,text='EXIT',font=('MS serif',12,'bold'),bg='gold',command=lambda:start(f1)).pack(side='bottom')
CT=Button(f12,bg='gold',font=('MS serif',12,'bold'),text='MAKE A NEW SELECTION',command=lambda:start(f2)).pack(side='bottom')



start(f1)
root.mainloop()
