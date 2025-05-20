# 0. Setting some variables

PROJECT_PATH="$( dirname -- "$BASH_SOURCE"; )";
PROJECT_PATH="$( realpath -e -- "$PROJECT_PATH/.."; )"
NFS_ROOT_PATH=/mnt/zynq/PDCv2
PYTHON_VENV_PATH=$PROJECT_PATH/venv_pdcv2
ZYNQ_USER=zynq # TBD
SUBNET_PREFIX="102.180.0"
USER_DATA_DIR=~/PDCv2-data

function print_usage {
echo "Run this script to setup your host PC to communicate with your devkit."
echo "Supplying no arguments will run all the steps, but you can also select specific ones by adding the followin cumulative options:"
echo "Options:"
echo "      --packages: install required packages for your OS"
echo "      --nfs_exports_dir: setup the directories that will be exported via NFS"
echo "      --append_bashrc: adds environment variables to your ~/.bashrc file"
echo "      --nfs_server_enable: enables and starts th NFS server on your host PC"
echo "      --python_env_creation: creates the python environment with required packages"
echo "      --eth_config: sets up the ethernet configuration to communicate with the ZCU102"
echo "      --ssh_rsa_config: configures ssh with a generated key to authenticate this PC with the ZCU102"
echo "      --create_data_dir: creates the data directory in your home folder"
echo "      --validate: validates PDC communication"
}

# 1. Install packages
function packages {
if [ -f "/etc/debian_version" ]; then # Running on Debian based OS
    echo "Running on Debian-based Linux"
    sudo apt update && sudo  apt install software-properties-common -y
    sudo add-apt-repository ppa:deadsnakes/ppa -y
    sudo apt install net-tools nfs-kernel-server python3.9 python3.9-venv python3.9-tk network-manager iproute2 -y
elif [ -f "/etc/redhat-release"  ]; then
    echo "Running on RHEL"
    sudo dnf update -y
    sudo dnf install net-tools python3.9 nfs-utils NetworkManager -y
fi
}

# 2. Place files in the appropriate places 
function nfs_exports_dir {
sudo mkdir -p $NFS_ROOT_PATH/bin 
sudo cp  $PROJECT_PATH/zynqApps/mnt/bin/* $NFS_ROOT_PATH/bin 
sudo mkdir -p $NFS_ROOT_PATH/data
sudo chmod 777 $NFS_ROOT_PATH/data
if [ -f "/etc/debian_version" ]; then # Running on Debian based OS
    sudo chown nobody:nogroup $NFS_ROOT_PATH/data
elif [ -f "/etc/redhat-release"  ]; then
    sudo chown nobody:nobody $NFS_ROOT_PATH/data
fi
}

# 3. Add environment variables and path to PATH in .bashrc
function append_bashrc {
if [ -f "/etc/debian_version" ]; then # Running on Debian based OS
    HOST_APPS_PATH=$PROJECT_PATH/hostApps/cpp/debianBasedOS/
elif [ -f "/etc/redhat-release"  ]; then
    HOST_APPS_PATH=$PROJECT_PATH/hostApps/cpp/redHatBasedOS/
fi

if ! grep -q PROJECT_PATH=$PROJECT_PATH ~/.bashrc; then
cat >> ~/.bashrc <<- EOL
export PROJECT_PATH=$PROJECT_PATH
export USER_DATA_DIR=$USER_DATA_DIR
export HDF5_DATA_DIR=$USER_DATA_DIR/HDF5
export PATH=$PATH:$HOST_APPS_PATH
EOL
fi
}

# 4. Enable NFS server
function nfs_server_enable {
if ! grep -q $NFS_ROOT_PATH /etc/exports; then
sudo sh -c " echo $NFS_ROOT_PATH       $SUBNET_PREFIX.4/24\(rw,subtree_check,sync\) >> /etc/exports"
fi
sudo systemctl restart nfs-kernel-server
sudo exportfs -a
}

# 5. Setup python environment
function python_env_creation {
python3.9 -m venv $PYTHON_VENV_PATH
. $PYTHON_VENV_PATH/bin/activate
pip install -r $PROJECT_PATH/hostApps/python/requirements.txt
}

# 6. Setup ethernet static IP on target interfaces
function eth_config {
echo "Available interfaces:"
ip addr show
read -p "Which do you want to use to communicate to ZCU102? (ex. enx001A)" interface
sudo nmcli con add con-name "static-$interface" ifname $interface type ethernet ip4 $SUBNET_PREFIX.1/24 
sudo nmcli con up static-$interface
}

# 7. Test communication to ZCU102
function ssh_rsa_config {
if [ ! -f ~/.ssh/id_rsa_zcu ]; then
ssh-keygen -t rsa -f ~/.ssh/id_rsa_zcu
fi
ssh-copy-id -i ~/.ssh/id_rsa_zcu.pub zynq@$SUBNET_PREFIX.16
if [ ! -f "~/.ssh/config" ]; then
    echo "SSH config not found, creating one"
    mkdir -p ~/.ssh/
    touch ~/.ssh/config
fi
if ! grep -q "Host zcudev" ~/.ssh/config; then
cat >> ~/.ssh/config <<- EOL
Host zcudev
  HostName $SUBNET_PREFIX.16
  User $ZYNQ_USER
  IdentityFile ~/.ssh/id_rsa_zcu
EOL
fi

ssh -q -o ConnectTimeout=5 zcudev exit
if [ $? = 0 ]; then
    echo "ZCU102 reached, you are good to go!"
else
    echo "Could not reach ZCU102, check you network configuration, cables and make sure the board is powered-on"
fi
}

# 8 Create HDF5 data folder in home
function create_data_dir {
mkdir -p $USER_DATA_DIR/HDF5
}


#################################################################
# Entrypoint:

if [[ $# -eq 0 ]] ; then
    packages # 1
    nfs_exports_dir # 2
    append_bashrc # 3
    nfs_server_enable # 4 
    python_env_creation # 5
    eth_config # 6
    ssh_rsa_config # 7
    create_data_dir # 8
else
    while [ $# -ne 0 ]
do
    arg="$1"
    case "$arg" in
        --packages)
            packages
            ;;
        --nfs_exports_dir)
            nfs_exports_dir
            ;;
        --append_bashrc)
            append_bashrc
            ;;
        --nfs_server_enable)
            nfs_server_enable
            ;;
        --python_env_creation)
            python_env_creation
            ;;
        --eth_config)
            eth_config
            ;;
        --ssh_rsa_config)
            ssh_rsa_config
            ;;
        --create_data_dir)
            create_data_dir
            ;;
        --help)
            print_usage
            ;;
        --*)
            nothing="true"
            ;;
    esac
    shift
done

fi
