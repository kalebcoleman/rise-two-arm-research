# GPU recovery on October 28, 2026

The workstation booted Ubuntu kernel `7.0.0-38-generic`, but NVIDIA's
precompiled kernel module was installed only for `7.0.0-31-generic`.
`lspci` detected the RTX 5080, `modinfo nvidia` failed, and `nvidia-smi`
could not communicate with the driver. Secure Boot was disabled.

With the owner's desktop administrator authentication, installed:

```bash
pkexec /usr/bin/apt-get --no-remove --no-install-recommends -y install \
  linux-modules-nvidia-580-open-7.0.0-38-generic \
  linux-modules-nvidia-580-open-generic-hwe-24.04=7.0.0-38.38~24.04.4+1
pkexec /sbin/modprobe nvidia_drm modeset=1
```

APT installed the current kernel's module and upgraded matching NVIDIA 580
components to `580.178.04`; no packages were removed and no newer kernel was
installed. Loading the module restored `nvidia-smi` enumeration of the RTX 5080
without rebooting. Isaac's Warp runtime subsequently detected `cuda:0` and
the physics test ran successfully.

These commands record this machine's repair, not a universal installation
recipe. On the school workstation first check its actual kernel, GPU and
installed driver. After future kernel updates, verify that the matching NVIDIA
module is installed before relying on a simulation session.
