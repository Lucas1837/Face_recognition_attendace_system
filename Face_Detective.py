#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sqlite3
import subprocess
import time 
from fpdf import FPDF
import numpy as np
import json

def register():    

    rospy.init_node("register_node")

    #connect to the database
    conn = sqlite3.connect("student_list.db")
    cur = conn.cursor()
    recognised_face = None

    section = input("Section :")
    Name = input("Name :")
    Matric_Num= input("Matric Number :")
    IC_NUMBER = input("IC NUMBER :")

    #run face_recognition node (change the path according to where you put the facial_recognition_attend file)
    process1 = subprocess.Popen("/home/lucas/catkin_ws/src/facial_attendance/facial_recognition_attend.py")

    def call_recognised(msg):
        nonlocal recognised_face 
        recognised_face = msg.data
 
    rospy.Subscriber("/recognized_faces" , String , callback = call_recognised )

    #wait until there is a face message subscribed
    while not recognised_face:
        rospy.Rate(0.5).sleep()
        rospy.loginfo("no face detected, please face to the camera")

    #If a face message is subscribed, terminate the face_recognition node
    if recognised_face :
        #function to calculate Euclideon distance between the face_encoding string in database with the one scanned 
        def compute_distance(face_id_array, json_array):

            try:
                target_array = np.array(json.loads(face_id_array))  
                parsed_array = np.array(json.loads(json_array)) 
                distance = np.linalg.norm(target_array - parsed_array)
                #If the distance is larger than 0.5 it will return None so that the query of database later gives None
                if distance >0.5 :
                    return None
                if distance <= 0.5:
                    return distance
                
            except (ValueError, TypeError) as e:
                print(f"Error in compute_distance: {e}")
                return float('inf') 
        #Creating the function in database
        conn.create_function("difference", 2, lambda json_array, _: compute_distance(recognised_face, json_array))
        #First select the record where distance is not None, and then rearrange the records From the least distance to largest
        cur.execute(f"""
        SELECT NAME,MATRIC_NUMBER,IC_NUMBER,difference(FACE_ID, '') AS diff
        FROM SECTION_{section}
        WHERE diff IS NOT NULL 
        ORDER BY diff
        LIMIT 1
        """) 
        #Obtain records selected
        exist_student = cur.fetchall()

        #define function to insert new data)
        def insert_in_table():
                cur.execute(f"""
                CREATE TABLE IF NOT EXISTS SECTION_{section}(
                NAME text,
                MATRIC_NUMBER text,
                IC_NUMBER text,
                FACE_ID text   
                        )""")
    
                cur.execute(f"INSERT INTO SECTION_{section} VALUES (?,?,?,?)" , (Name,Matric_Num,IC_NUMBER,recognised_face))
                print(f"Student : {Name} with Matric Number : {Matric_Num}, IC : {IC_NUMBER} sucessfuly registered")

        #check whether there is a record with similar face encoding
        if exist_student:
            rospy.loginfo(f"STUDENT : {exist_student[0][0]} , with Matric_Number = {exist_student[0][1]} , IC_Number = {exist_student[0][2]} , has already exist in the table")
            question = input("""
Stil want to register?
type 1 for yes, 2 for no
------->""")
            if question == "1":
                insert_in_table()

        else:
            insert_in_table()

    conn.commit()
    conn.close
    process1.terminate()
    time.sleep(3)

def taking_attendance():
    rospy.init_node("Attendance_node")

    #run tts_node (change the path according to where you put the tts file)
    process_2 = subprocess.Popen("/home/lucas/catkin_ws/src/facial_attendance/tts.py")

    time.sleep(1.5)

    pub = rospy.Publisher("/name",String,queue_size=10)

    name_of_student = String()
    attended_students= []
    attended_students_name = []
    not_attended_students = []
    face_id = None

    conn = sqlite3.connect("student_list.db")
    cur = conn.cursor()

    section = input("SECTION :")
    date = input("Date (type without space): ")

    #run face_recognition node
    process_1 = subprocess.Popen("/home/lucas/catkin_ws/src/facial_attendance/facial_recognition_attend.py")

    #callback funtion to obtain the face_encoding publish by face_recognition node
    def call_recognised(msg):
        nonlocal face_id 
        face_id = msg.data

    rospy.Subscriber("/recognized_faces" , String , callback = call_recognised )    

    #wait until a face is detected
    while not face_id:
            rospy.loginfo("No faces detected")
            rospy.Rate(0.3).sleep()
            
    while face_id:
        def compute_distance(face_id_array, json_array):

            try:
                target_array = np.array(json.loads(face_id_array))  
                parsed_array = np.array(json.loads(json_array)) 
                distance = np.linalg.norm(target_array - parsed_array)  
                #the threshold distance here is set smaller in order to minimise the chance of scanning face from phone screen 
                if distance > 0.28:
                    return None
                if distance <= 0.28 :
                    return distance
                
            except (ValueError, TypeError) as e:
                print(f"Error in compute_distance: {e}")
                return float('inf')  

        conn.create_function("difference", 2, lambda json_array, _: compute_distance(face_id, json_array))
        
        cur.execute(f"""
        SELECT NAME,MATRIC_NUMBER,IC_NUMBER,difference(FACE_ID, '') AS diff
        FROM SECTION_{section}
        WHERE diff IS NOT NULL 
        ORDER BY diff
        LIMIT 1
        """) 
        #obtain the record with the least distance
        attended = cur.fetchone()
        
        if attended is not None:
            #slicing for the first 3 tuples from the list, Name, Matric_Num, IC_Num
            student_info = attended[0:3]
            #check if the currently obtained student info already exist in the list of attended_student

            if student_info not in attended_students:
                attended_students.append(student_info)
                attended_students_name.append(student_info[0])
                name_of_student.data = student_info[0]
                pub.publish(name_of_student)

        if not attended:
            rospy.loginfo("No faces detected")
        #check if the face_recognition_node is terminated
        if process_1.poll() is not None:
            #terminate the tts_node
            process_2.terminate()
            break


        rospy.Rate(1).sleep()
    #Introduce variable of placeholder that will be used to select record base on how many students record in attended_students
    placeholders = ', '.join(['?'] * len(attended_students))

    #Query for not_attended student by selecting record that do not match with the attended_students
    cur.execute(f"""
                SELECT NAME,MATRIC_NUMBER,IC_NUMBER FROM SECTION_{section}
                WHERE NAME NOT IN ({placeholders})
    """ ,(attended_students_name)) 

    not_attended = cur.fetchall()

    for record2 in not_attended:
        not_attended_students.append(record2)

    print(f"""attended_students : 
    {attended_students}
          """)
    
    print(f"""not_attended_students : 
    {not_attended_students}
          """)

    conn.commit
    conn.close

    #CREATING PDF#
    pdf = FPDF('P','mm','A4')
    pdf.add_page()
    #B=BOLD,U=Underline,12=word size    
    pdf.set_font('Arial' , 'BU', 12 ) 
    #0=cell width extend till end,3=height of cell,0=no boarder,ln = 1 means the cursor(pointer) go to the next line,c=center the word in the cell    
    pdf.cell(0,3,f"SECTION-{section}-{date}-ATTENDED",0, ln =1 ,align="C")
    #cursor go to the next line
    pdf.ln() 
    pdf.set_font('Arial' , 'B' ,12 )
    pdf.cell(63,10 ,"NAME" , border=1)
    pdf.cell(63,10 ,"MATRIC_NUMBER" , border =1)
    pdf.cell(63,10 ,"IC_NUMBER" , border=1, ln =1)
    #for loop to create rows according to the number of records
    for record in attended_students:
        if pdf.get_y() + 10 > pdf.h: #get the current cursor's y position , if after adding new column with height 10 it exceed the height of page, add new page
            pdf.add_page()
        #get the x,y position of the cursor right now
        x1=pdf.get_x()
        y1=pdf.get_y()
        pdf.set_font('Arial' , '' ,12 )# '' stands for regular form of word
        #multi_cell will automatically 
        pdf.multi_cell(63,8 ,record[0] , border =1)
        y2 = pdf.get_y()
        pdf.set_xy(x1+63 , y1)
        pdf.cell(63,y2-y1 ,record[1] , border = 1)
        pdf.cell(63,y2-y1 ,record[2] , border = 1 ,ln =1)
    #Adding pages for not attended students, the following just the same cmoomand as above but just for not_attended_students
    pdf.add_page()
    pdf.set_font('Arial' , 'BU', 12 ) 
    pdf.cell(0,3,f"SECTION-{section}-{date}-NOT-ATTENDED",0, ln =1 ,align="C")
    pdf.ln() 
    pdf.set_font('Arial' , 'B' ,12 )
    pdf.cell(63,10 ,"NAME" , border=1)
    pdf.cell(63,10 ,"MATRIC_NUMBER" , border =1)
    pdf.cell(63,10 ,"IC_NUMBER" , border=1, ln =1)

    for record in not_attended_students:
        if pdf.get_y() + 10 > pdf.h:
            pdf.add_page()
        x1=pdf.get_x()
        y1=pdf.get_y()
        pdf.set_font('Arial' , '' ,12 )
        pdf.multi_cell(63,8 ,record[0] , border =1)
        x2 = pdf.get_x()
        y2 = pdf.get_y()
        pdf.set_xy(x1+63 , y1)
        pdf.cell(63,y2-y1 ,record[1] , border = 1)
        pdf.cell(63,y2-y1 ,record[2] , border = 1 ,ln =1)
    file_name = f"SECTION_{section}_{date}_ATTENDANCE.pdf"
    pdf.output(file_name)    
    subprocess.run(["xdg-open", f"SECTION_{section}_{date}_ATTENDANCE.pdf"])#this will open the pdf 


Action = input("""
1.Register 
2.Taking Attendance 
(type 1 or 2)
--->""")

if Action == "1":
    register()

elif Action == "2":
    taking_attendance()

else:
    print("PLEASE CHOOSE EITHER ONE")