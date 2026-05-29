# System Status
- Host: robot-controller.local
- User: snoopy
- Access with:
```bash
ssh snoopy@robot-controller
``` 
(or ssh snoopy@robot-controller.local without DNS Server)
Copy ssh key to remote machine for no-pw on login:
```bash
ssh-copy-id snoopy@robot-controller
```

# Robot Controller Setup Guide
This document logs the step-by-step installation and configuration process of transforming a dedicated Mini-PC into a real-time capable robot controller running Ubuntu Server.
### 1. Installation Media Preparation
- OS Image: Downloaded the official Ubuntu Server ISO (LTS version).
- Bootable Drive: Flashed the ISO onto a USB thumb drive using a tool.

### 2. Ubuntu Server Installation Phase
During the initial boot from the USB drive, the following critical installation choices were made in the setup process:

- Storage Configuration: Selected the option to utilize the entire local SSD to ensure maximum storage capacity and clean partition layouts.
- Network Connectivity: The device should be connected to a working network during the installation process, or Ubuntu's network configuration framework (Netplan/systemd-networkd) will fail to setup properly.
- SSH Server: Explicitly checked the box to install and enable the OpenSSH Server during the wizard. This eliminates the need for a permanent monitor and keyboard on the Mini-PC after installation.

# Realtime Robot Controller Setup
The following steps must be taken to meet real-time requirements and run ros software.

### Upgrading to a Real-Time Kernel (PREEMPT_RT)
To ensure highly deterministic execution loops required for low-level motor controls and sensor processing, the standard kernel was upgraded to a Hard Real-Time Kernel.
Steps Taken:

1. Ubuntu Pro Attachment: Registered a free personal account on ubuntu.com/pro and linked the machine via:
```bash
sudo pro attach <YOUR_TOKEN>
```
2. Enabling Real-Time: Executed the real-time installation command:
```bash
sudo pro enable real-time-kernel
```

### Disabling Power Saving & Sleep Modes
To guarantee that the robot controller remains 100% operational and accessible at all times, all OS-level suspension and sleep targets were permanently masked. This prevents the Mini-PC from entering any power-saving states during headless execution.
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

###  Docker Setup

This project runs inside Docker containers to ensure environment consistency and easy deployment across different machines.
Install Docker using the native `docker.io` package:

```bash
sudo apt install -y docker.io
```
By default, Docker commands require root privileges (sudo). To run Docker as a non-root user, add your current user to the docker group:
```bash
sudo usermod -aG docker $USER
```
> Important: After running this command, log out and log back in (or restart your terminal / SSH session) for the changes to take effect!

To achieve deterministic execution and allow ros2_control to run high-priority real-time loops inside a Docker container, you must configure both the Dockerfile and the Runtime Flags:
#### Container Runtime Flags (Crucial)
A standard Docker container restricts real-time scheduling (SCHED_FIFO / SCHED_RR) for security reasons. To grant your container the ability to utilize the host's PREEMPT_RT capabilities, you must launch it with elevated privileges and real-time resource limits (ulimits).

When running your container, add the following flags:
```bash
docker run -it \
  --privileged \
  --net=host \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  your_rt_image_name
```
What these flags do:
- --privileged: Grants the container access to host hardware interfaces (e.g., EtherCAT, CAN, USB).
- --net=host: Bypasses Docker's network bridge to eliminate latency spikes in ROS 2 communication.
- --ulimit rtprio=99: Allows the ros2_control threads to set the highest possible real-time priority (99).
- --ulimit memlock=-1: Allows unlimited memory locking (mlockall()), preventing the Linux kernel from swapping out critical real-time memory pages to disk.


### Remote Development Setup

This repository is optimized for remote development on the `robot-controller` using VS Code and Git. 

1. Git Configuration
Before making any changes, ensure your Git identity is configured directly on the remote server:

```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

2. VS Code Remote Development

You can develop directly on the remote machine using your local VS Code via SSH. No manual server setup is required.

- Install Extension: Open VS Code on your laptop, go to Extensions (Ctrl+Shift+X), and install Remote - SSH by Microsoft.

- Connect to Host:

    - Click the green/blue >< (Remote Window) icon in the bottom-left corner of VS Code.
    - Select Connect to Host... -> Add New SSH Host...
    - Enter the connection string: snoopy@robot-controller (or use the server's IP address).

- Open Workspace: Once connected, click Open Folder in the Explorer sidebar and select your project directory (e.g., /home/snoopy/your-project).

> Tip: Since your SSH key is already configured, VS Code will connect instantly without prompting for a password. The integrated terminal (Ctrl + ~) will also open directly on the remote server.