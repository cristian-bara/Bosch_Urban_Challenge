from sys import argv
import os

script, compName = argv

print compName

if not os.path.exists(compName):
    os.makedirs(compName)
else:
    print "component already exists"