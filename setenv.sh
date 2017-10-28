#!/bin/sh
# The Ultiparc Project.
# Environment configuration.

echo ""
echo "Setting environment for Ultiparc."
echo "================================="
echo ""

# Set work directory
export ULTIPARC_HOME=`pwd`
echo "Workspace: $ULTIPARC_HOME"

# Check that SYSTEMC_HOME variable is set.
if [ -z ${SYSTEMC_HOME+x} ]; then
   echo "SystemC: Warning! SYSTEMC_HOME variable is not unset.";
else
   echo "SystemC: SYSTEMC_HOME = $SYSTEMC_HOME";
fi

echo ""
