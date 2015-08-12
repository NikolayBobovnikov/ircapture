#!/usr/bin/env python

import sys, os, commands, time, errno

def getExecutablesAndMd5(directory):
    """
    Returns a dictionary with the following.
    Key: paths ti execytable within specified directory.
    Value: the md5sums of the exwcutable.
    """

    execs = {}
    for d in os.listdir(directory):
        filePath = os.path.join(directory,d)
        if (os.path.isfile(filePath) and os.access(filePath, os.X_OK)):
            execs[filePath] = commands.getoutput('md5sum ' + filePath)
    return execs

def sanityCheck():
    if len(sys.argv) < 2:
        print 'Not enough arguments. Specify directory to monitor.'
        exit(errno.ENOTDIR)

def main():
    sanityCheck()

    os.system('clear')

    print 'Waiting for executables.'

    monitorDir = str(sys.argv[1])
    execs = getExecutablesAndMd5(monitorDir)
    loopsSinceClear = 0

    while (True):
        execsNow = getExecutablesAndMd5(monitorDir)
        #Loop through to see if tests should be run.
        areThereTestsToRun = False
        for k in execsNow:
            isNew = not execs.has_key(k)
            isUpdated = execs.has_key(k) and execsNow[k] != execs[k]
            if isNew or isUpdated:
                areThereTestsToRun = True

        if (areThereTestsToRun):
        #Only clear screen if >6 iretations have passed by
        #without tests being modified.
            if loopsSinceClear > 6:
                os.system('clear')
                loopsSinceClear = 0

                #Execute tests that are new or updated.
                for k in execsNow:
                    isNew = not execs.has_key(k)
                    isUpdated = execs.has_key(k) and execsNow[k] != execs[k]
                    if isNew or isUpdated:
                        print commands.getoutput(k)
            else:
                loopsSinceClear += 1

            execs = execsNow
            time.sleep(0.5);

if __name__ == "__main__":
    main()

