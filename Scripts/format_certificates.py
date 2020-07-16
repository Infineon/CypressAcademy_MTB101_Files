'''
Python Script to format certificates for aws_config.h file
for running AWS Publisher/Subscriber code example on Mbed OS

Developed on Python 3.7.4

Developed by Varun Kaushik
'''

import os

path = os.path.dirname(os.path.realpath(__file__))

#Function that adds a new line character and trailing backslash except on the final line
def add_newline(f):
    
    with open(f, 'r') as fd:
        lines = fd.read().splitlines()
    if(f.endswith("private.pem.key")):
       print("/* Thing Private Key */")
    elif(f.endswith("certificate.pem.crt")):
         print("/* Thing Certificate */")
    elif(f.endswith("CA1.pem")):
         print("/* Amazon Root Certificate */")
    else:
        return
    line_num = 0
    for i in lines:
        i = "\""+i+"\\n\""
        line_num = line_num + 1
        if(len(lines) == line_num):
            print(i)
        else:
            print(i+"\\")


#Main function. Execution starts here
if __name__ == '__main__':

    files = os.listdir(path)
    for f in files:
        if (f.endswith(".crt") or f.endswith(".pem") or f.endswith(".pem.key")):
            #print ("File: "+f)
            add_newline(f)
            print ("")
            
input()




