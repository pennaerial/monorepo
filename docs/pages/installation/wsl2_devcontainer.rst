Windows Devcontainer Installation 
====================

.. rst-class:: lead

   Installation Guide for Windows using WSL2 and Docker Devcontainer.
----

Prerequisites
-------------
 
- Windows 11 (And probably Windows 10 21H2+)
- An NVIDIA GPU

Set Up WSL2
-------------------

Install/Update WSL2 and Ubuntu 24.04:

.. code-block:: powershell
   :caption: PowerShell
 
   wsl --update
   wsl --install -d Ubuntu-24.04
 
In WSL, verify that your GPU is visible:

.. code-block:: bash
   :caption: Bash
 
   nvidia-smi -L
 
You should see your NVIDIA GPU listed.
 
Install Docker Engine inside WSL
---------------------------------
 
.. code-block:: bash
   :caption: Bash
 
   curl -fsSL https://get.docker.com | sh
   sudo usermod -aG docker $USER
 
.. note::
 
   Ignore the warning recommending Docker Desktop, just wait 20 seconds

Close and reopen your WSL terminal for the group change to apply.
 
Install the NVIDIA Container Toolkit
--------------------------------------
 
.. code-block:: bash
   :caption: Bash
 
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
   curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   sudo apt update
   sudo apt install -y nvidia-container-toolkit
 
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
 
.. note::
 
   If ``systemctl restart docker`` errors out, try
   ``sudo service docker restart`` instead
 
Verify:
 
.. code-block:: bash
   :caption: Bash
 
   docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi

Get a GitHub Personal Access Token (PAT)
-------------------------------------------
 
The monorepo Docker image is private, so you need a token to pull it.
 
#. Go to https://github.com/settings/tokens → **Generate new token (classic)**
#. Check only the ``read:packages`` scope
#. Generate
#. Copy the token
 
Log in with it (paste the PAT when prompted for password):
 
.. code-block:: bash
   :caption: Bash
 
   docker login ghcr.io -u <your-github-username>

.. warning::
   
   Docker will save your PAT in an unencrypted file.
 
 
Clone the Repo
---------------
 
.. code-block:: bash
   :caption: Bash
 
   git clone --recurse-submodules https://github.com/pennaerial/monorepo.git
   cd monorepo
 
Install VS Code
---------------------------------------------
 
- Install `VS Code <https://code.visualstudio.com/>`_ if you don't have it
- Install the **Dev Containers** extension (``ms-vscode-remote.remote-containers``)
- Open the repo in WSL: from your WSL terminal, ``code .``
 
Open in Container
-------------------
 
With the repo open in VS Code, open the Command Palette
(``Ctrl+Shift+P``) → **"Dev Containers: Reopen in Container"**.
 
VS Code will pull the image and apply the devcontainer config. First
run installs a few packages and can take a long time.
 
Verify it's Working
----------------------
 
In the VS Code integrated terminal (now inside the container):

.. code-block:: bash
   :caption: Bash

   nvidia-smi -L
   glxinfo | grep "OpenGL renderer"   # should show your NVIDIA GPU, not llvmpipe

Then try Gazebo:
 
.. code-block:: bash
   :caption: Bash
 
   cd $PENNAIR_PX4_PATH
   make px4_sitl gz_x500
 
A Gazebo window should appear on your desktop, and be running smoothly.
 
Troubleshooting
------------------
 
.. list-table::
   :header-rows: 1
   :widths: 40 60
 
   * - Error
     - Fix
   * - ``OpenGL renderer`` shows your integrated GPU instead of NVIDIA
     - Set ``MESA_D3D12_DEFAULT_ADAPTER_NAME`` to match your discrete
       GPU's name
   * - ``git rev-parse --verify HEAD`` fails during build
     - Run ``git config --global --add safe.directory '*'`` inside the
       container
   * - ``docker pull`` fails with auth error
     - Re-run ``docker login ghcr.io``, your PAT may have expired
 

