import numpy as np
import math
import PySimpleGUI as sg
import pandas as pd

# GUI code

sg.theme('DarkBlue')

# Excel read code

EXCEL_FILE =  '3-DOF Cartesian Manipulator Design Data.xlsx'
df = pd.read_excel(EXCEL_FILE)

# Lay-out code

layout = [
    [sg.Text('Fill out the following fields')],
    [sg.Text('a1 = '),sg.InputText(key='a1', size=(20,10)),sg.Text('d1 = '),sg.InputText(key='d1', size=(20,10))],
    [sg.Text('a2 = '),sg.InputText(key='a2', size=(20,10)),sg.Text('d2 = '),sg.InputText(key='d2', size=(20,10))],
    [sg.Text('a3 = '),sg.InputText(key='a3', size=(20,10)),sg.Text('d3 = '),sg.InputText(key='d3', size=(20,10))],
    [sg.Text('a4 = '),sg.InputText(key='a4', size=(20,10))],
    [sg.Button('Solve Forward Kinematics')],
    [sg.Frame('Position Vector: ',[[
          sg.Text('X = '),sg.InputText(key='X', size=(10,1)),
          sg.Text('Y = '),sg.InputText(key='Y', size=(10,1)),
          sg.Text('Z = '),sg.InputText(key='Z', size=(10,1))]])],
    [sg.Frame('H0_3 Transformation Matrix = ',[[sg.Output(size=(60,10))]])],
    [sg.Submit(), sg.Button('Clear Input'),sg.Exit()]      

       
    ]

window = sg.Window('Cartesian Manipulator',layout)

def clear_input():
    for key in values:
        window[key]('')
    return None

while True:
    event,values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit':
        break  
    if event == 'Clear Input':
        clear_input()
    if event == 'Solve Forward Kinematics':
       
        # Forward Kinematic Codes
        # link lengths in cm
        a1 = values['a1']
        a2 = values['a2']
        a3 = values['a3']
        a4 = values['a4']

        # Joint Variable Thetas in degrees
        d1 = values['d1']
        d2 = values['d2']
        d3 = values['d3']

        # If Joint Variable are ds don't need to convert

        ## D-H Parameter Table (This is the only part you only edit for every new mechanical manipulator.)
        # Rows = no. of HTM, Colums = no. of Parameters
        # Theta, alpha, r, d

        DHPT = [[0,(270.0/180.0)*np.pi,0,float(a1)],
                [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,float(a2)+float(d1)],
                [(270.0/180.0)*np.pi,(90.0/180.0)*np.pi,0,float(a3)+float(d2)],
                [0,0,0,float(a4)+float(d3)]
                ]

        # np.trigo function (DHPT[row][column])

        i = 0
        H0_1 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                 [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                 [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                 [0,0,0,1]]

        i = 1
        H1_2 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                 [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                 [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                 [0,0,0,1]]

        i = 2
        H2_3 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                 [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                 [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                 [0,0,0,1]]

        i = 3
        H3_4 = [[np.cos(DHPT[i][0]),-np.sin(DHPT[i][0])*np.cos(DHPT[i][1]),np.sin(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.cos(DHPT[i][0])],
                 [np.sin(DHPT[i][0]),np.cos(DHPT[i][0])*np.cos(DHPT[i][1]),-np.cos(DHPT[i][0])*np.sin(DHPT[i][1]),DHPT[i][2]*np.sin(DHPT[i][0])],
                 [0,np.sin(DHPT[i][1]),np.cos(DHPT[i][1]),DHPT[i][3]],
                 [0,0,0,1]]

        # Transportation Matrices from base to end-effector
        #print("H0_1 = ")
        #print(np.matrix(H0_1))
        #print("H1_2 = ")
        #print(np.matrix(H1_2))
        #print("H2_3 = ")
        #print(np.matrix(H2_3))

        # Dot Product of H0_3 = H0_1*H1_2*H2_3
        H0_2 = np.dot(H0_1,H1_2)
        H0_3 = np.dot(H0_2,H2_3)
        H0_4 = np.dot(H0_3,H3_4)

        # Transportation Matrix of the Manipulator
        print("H0_4 = ")
        print(np.matrix(H0_4))

        # Position Vector X Y Z
        
        X0_4 = H0_4[0,3]
        print("X = ", X0_4)

        Y0_4 = H0_4[1,3]
        print("Y = ", Y0_4)

        Z0_4 = H0_4[2,3]
        print("Z = ", Z0_4)

    if event == 'Submit':
        df = df.append(values, ignore_index=True)
        df.to_excel(EXCEL_FILE, index=False)    
        sg.popup('Data saved!')
        clear_input()  

window.close()