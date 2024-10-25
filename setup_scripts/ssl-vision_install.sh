#Install SSL-Vision repo and dependencies.
cd ..
cd ..
mkdir ssl-vision
cd ssl-vision
git clone https://github.com/RoboCup-SSL/ssl-vision.git
cd ssl-vision
sudo chmod +x InstallPackagesUbuntu.sh
./InstallPackagesUbuntu.sh
make