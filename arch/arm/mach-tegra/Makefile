GCOV_PROFILE := y


obj-y                                   += ahb.o
obj-y                                   += apbio.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= board-pinmux.o
obj-y                                   += common.o
obj-y                                   += board-info.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += common-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += common-t3.o
obj-y                                   += io.o
obj-y                                   += irq.o
obj-y                                   += clock.o
obj-y                                   += clock-common.o
obj-y                                   += timer.o
obj-y                                   += timerinfo.o
obj-y                                   += pinmux.o
obj-y                                   += devices.o
obj-y                                   += delay.o
obj-y                                   += pm.o
obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
obj-$(CONFIG_PM_SLEEP)                  += pm-irq.o
obj-y                                   += gic.o

obj-y                                   += sleep.o

obj-y					+= fuse.o
obj-y					+= powergate.o
obj-y					+= flowctrl.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_clocks.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra30_clocks.o
ifeq ($(CONFIG_ARCH_TEGRA_2x_SOC),y)
obj-y                                   += timer-t2.o
else
obj-y                                   += timer-t3.o
endif
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= tegra2_emc.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= tegra3_emc.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= pinmux-tegra20-tables.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= pinmux-tegra30-tables.o

# obj-$(CONFIG_ARCH_TEGRA_3x_SOC)		+= board-dt-tegra30.o


plus_sec := $(call as-instr,.arch_extension sec,+sec)
AFLAGS_sleep.o :=-Wa,-march=armv7-a$(plus_sec)

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += sleep-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += sleep-t3.o
obj-y                                   += kfuse.o
obj-y                                   += csi.o
obj-y    				+= tegra_odm_fuses.o
obj-y                                   += i2c_error_recovery.o
obj-y                                   += mc.o
obj-$(CONFIG_TEGRA_STAT_MON)            += tegra2_statmon.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra3_usb_phy.o
obj-$(CONFIG_USB_SUPPORT)               += tegra2_usb_phy.o
obj-$(CONFIG_FIQ)                       += fiq.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o
obj-$(CONFIG_TEGRA_PWM)                 += pwm.o
obj-$(CONFIG_TEGRA_ARB_SEMAPHORE)       += arb_sema.o

obj-y                                   += dvfs.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += latency_allowance.o
obj-$(CONFIG_TEGRA_EDP_LIMITS)          += edp.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_speedo.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_actmon.o

obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += wakeups-t3.o
obj-y                                   += wakeups.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += pm-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += pm-t3.o

obj-$(CONFIG_SMP)                       += platsmp.o
obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
obj-y                                   += headsmp.o
obj-y                                   += reset.o
obj-$(CONFIG_TEGRA_SYSTEM_DMA)          += dma.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
ifeq ($(CONFIG_TEGRA_AUTO_HOTPLUG),y)
ifeq ($(CONFIG_CPUQUIET_FRAMEWORK),y)
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuquiet.o
else
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpu-tegra3.o
endif
endif
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o
obj-$(CONFIG_USB_SUPPORT)               += usb_phy.o
ifeq ($(CONFIG_CPU_IDLE),y)
obj-y                                   += cpuidle.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += cpuidle-t2.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += cpuidle-t3.o
endif
endif
ifeq ($(CONFIG_TEGRA_THERMAL_THROTTLE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_throttle.o
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_throttle.o
endif
obj-$(CONFIG_ARCH_TEGRA_3x_SOC)         += tegra3_thermal.o
obj-$(CONFIG_TEGRA_IOVMM)               += iovmm.o
obj-$(CONFIG_TEGRA_IOVMM_GART)          += iovmm-gart.o
obj-$(CONFIG_TEGRA_IOVMM_SMMU)          += iovmm-smmu.o
obj-$(CONFIG_DEBUG_ICEDCC)              += sysfs-dcc.o
obj-$(CONFIG_TEGRA_CLUSTER_CONTROL)     += sysfs-cluster.o
ifeq ($(CONFIG_TEGRA_MC_PROFILE),y)
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_mc.o
endif
obj-$(CONFIG_SENSORS_TEGRA_TSENSOR)     += tegra3_tsensor.o
obj-$(CONFIG_TEGRA_DYNAMIC_PWRDET)      += powerdetect.o
obj-$(CONFIG_TEGRA_USB_MODEM_POWER)     += tegra_usb_modem_power.o
obj-$(CONFIG_TEGRA_PCI)                 += pcie.o

obj-$(CONFIG_MACH_TEGRA_DT)             += board-dt-tegra20.o
obj-$(CONFIG_MACH_TEGRA_DT)             += board-harmony-pinmux.o

obj-$(CONFIG_MACH_TEGRA_DT)             += board-seaboard-pinmux.o
obj-$(CONFIG_MACH_TEGRA_DT)             += board-paz00-pinmux.o
obj-$(CONFIG_MACH_TEGRA_DT)             += board-trimslice-pinmux.o

obj-$(CONFIG_MACH_CARDHU)               += board-cardhu.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-kbc.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-irda.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-panel.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-pinmux.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-power.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-pm298-power-rails.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-pm299-power-rails.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-sdhci.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-sensors.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-memory.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-powermon.o
obj-$(CONFIG_MACH_CARDHU)               += board-cardhu-misc.o
obj-$(CONFIG_MACH_CARDHU)               += vibrator.o

obj-y                                   += board-touch-raydium_spi.o

obj-$(CONFIG_TEGRA_BB_XMM_POWER)        += baseband-xmm-power.o
obj-$(CONFIG_TEGRA_BB_XMM_POWER2)       += baseband-xmm-power2.o

