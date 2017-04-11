from sys import argv
import os

script, compName = argv

print compName

if not os.path.exists(compName):
    os.makedirs(compName)
    file = open(compName + "\\" + compName + ".h", 'w')
    file.writelines("#ifndef " + compName.upper() + "_H\n")
    file.writelines("#define " + compName.upper() + "_H\n")
    file.writelines("#include <" + compName +".inl>\n")
    file.writelines("#endif // " + compName.upper() + "_H\n")
    file.close()
    file = open(compName + "\\" + compName + ".inl", 'w')
    file.writelines("\n")
    file.close()
    file = open(compName + "\\" + compName + ".cpp", 'w')
    file.writelines("#include <" + compName +".h>\n")
    file.close()

else:
    print "component already exists"