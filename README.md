//cd <kernel_source_code_location>
cd ~/Linux_for_Tegra/source/kernel/kernel-jammy-src
git init
git remote add origin https://github.com/sohyunkim6572/ros2_framework_kernel.git
git remote -v
git fetch --all
git reset --hard origin/main
git pull origin main
