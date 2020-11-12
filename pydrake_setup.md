# Installing pydrake
```bash
curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-bionic.tar.gz 
sudo tar -xzf drake.tar.gz -C /opt 
sudo apt-get update -o APT::Acquire::Retries=4 -qq 
sudo apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy --no-install-recommends $(cat /opt/drake/share/drake/setup/packages-bionic.txt)
```

Then, add '/opt/drake/lib/python3.6/site-packages' to your system variable PYTHONPATH. (optional - if using PyCharm can skip)

# Configuring PyCharm
If using PyCharm (without a virtual environment):
- Open the project
- Add a configuration on base Python 3.6
- Edit the 'Environment variables' field
- Add variable 'PYTHONPATH' with value '/opt/drake/lib/python3.6/site-packages'
