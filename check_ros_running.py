import subprocess, shlex

def check_ros_running():
    ps = subprocess.Popen(shlex.split("ps auxw"), stdout=subprocess.PIPE)
    grep = subprocess.check_output(shlex.split("grep ros"), stdin=ps.stdout)

    split = grep.decode().split("\n")

    # print(split)
    return len(split) > 3 # This program + grep + blank line

if __name__ == "__main__":
    print(check_ros_running())
