# JetsonCar -- Python tools
Code, libraries and other tools built for the Jetson RC car project but does not fit in the other repositories, eg. MPC code generators, log-processing etc.

## Python environment (Anaconda)
Install Anaconda.
Build the `cpp` folder in `tools` to download and install binary depdencies (e.g. `acados`).
Next we will create an environment named `jetsoncar` and install all the required packages.

```bash
conda create -y --name jetsoncar python=3.7
conda activate jetsoncar
pip3 install -e ~/repos/acados/interfaces/acados_template 
#conda install --force-reinstall -y -q --name jetsoncar -c conda-forge --file requirements.txt
pip3 install -r requirements.txt

cd $CONDA_PREFIX
conda deactivate
mkdir -p ./etc/conda/activate.d
mkdir -p ./etc/conda/deactivate.d
touch ./etc/conda/activate.d/env_vars.sh
touch ./etc/conda/activate.d/pip.conf
touch ./etc/conda/deactivate.d/env_vars.sh
cat >> ./etc/conda/activate.d/env_vars.sh <<EOF
#!/bin/sh
export OLD_PYTHONPATH=\$PYTHONPATH
export PYTHONPATH=\$PYTHONPATH

# Absolute path this script is in
SCRIPTPATH="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" && pwd )"
echo \$SCRIPTPATH

export OLD_LDLIBRARYPATH=\$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:"~/repos/acados/lib"

export ACADOS_SOURCE_DIR="~/repos/acados"
EOF
cat >> ./etc/conda/deactivate.d/env_vars.sh <<EOF
#!/bin/sh
export PYTHONPATH=\$OLD_PYTHONPATH
export LD_LIBRARY_PATH=\$OLD_LDLIBRARYPATH
EOF
```

## Notes
