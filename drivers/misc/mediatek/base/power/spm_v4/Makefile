ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/spm_v4/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/spm_v4/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/eem_v1/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/mach/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/irq/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dramc/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/thermal/fakeHeader/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic_wrap/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mach/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6356/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/videox/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/dispsys/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/$(MTK_PLATFORM)/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/video/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v3/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/cmdq/v3/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/lcm/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/hps_v3/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/mcdi/include
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/include/
ccflags-y += -I$(srctree)/drivers/scsi/ufs/

ifeq ($(CONFIG_MACH_MT6771), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/submodule/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/emi/$(MTK_PLATFORM)/
endif

ifeq ($(CONFIG_MACH_MT6771), y)
ccflags-y  += -I$(srctree)/drivers/misc/mediatek/thermal/$(MTK_PLATFORM)/inc/
endif

ifeq ($(CONFIG_MTK_SND_SOC_NEW_ARCH), y)
ccflags-y += -I$(srctree)/sound/soc/mediatek/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/sound/soc/mediatek/common_int/
endif

ifeq ($(CONFIG_MTK_TINYSYS_SSPM_SUPPORT), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sspm/$(MTK_PLATFORM)/
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/
ccflags-y += -I$(srctree)/drivers/devfreq/

ifeq ($(CONFIG_MACH_MT6771), y)
obj-y += mtk_idle.o
obj-y += mtk_sleep.o
obj-y += mtk_spm_init.o
obj-y += mtk_spm.o
obj-y += mtk_spm_dpidle.o
obj-y += mtk_idle_profile.o
obj-y += mtk_spm_fs.o
obj-y += mtk_spm_internal.o
obj-y += mtk_spm_resource_req.o
obj-y += mtk_spm_sleep.o
obj-y += mtk_spm_sodi.o
obj-y += mtk_spm_sodi3p0.o
obj-y += mtk_spm_vcorefs_mt6771.o
ifeq ($(CONFIG_MTK_TINYSYS_SSPM_SUPPORT), y)
obj-y += mtk_spm_vcorefs_ipi_mt6771.o
endif
else
obj-y += mtk_idle.o
obj-y += mtk_sleep.o
obj-y += mtk_spm_init.o
obj-y += mtk_spm.o
obj-y += mtk_spm_dpidle.o
obj-y += mtk_idle_profile.o
obj-y += mtk_spm_fs.o
obj-y += mtk_spm_internal.o
obj-y += mtk_spm_resource_req.o
obj-y += mtk_spm_sleep.o
obj-y += mtk_spm_sodi.o
obj-y += mtk_spm_sodi3p0.o
obj-y += mtk_spm_vcorefs.o
endif

ifeq ($(CONFIG_MACH_MT6763), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/cpuidle_v2/
obj-y += mtk_idle_mt6763.o
obj-y += mtk_spm_dpidle_mt6763.o
ifneq ($(CONFIG_MTK_TINYSYS_SSPM_SUPPORT), y)
obj-y += mtk_spm_pmic_wrap.o
endif
obj-y += mtk_spm_sleep_mt6763.o
obj-y += mtk_spm_sodi_mt6763.o
endif

ifeq ($(CONFIG_MACH_MT6739), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/clkbuf_v1/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/eem_v2/$(MTK_PLATFORM)/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/cpuidle_v3/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/cpufreq_v2/src/mach/$(MTK_PLATFORM)/
obj-y += mtk_idle_mt6739.o
obj-y += mtk_spm_dpidle_mt6739.o
obj-y += mtk_spm_sleep_mt6739.o
obj-y += mtk_spm_sodi_mt6739.o
endif

ifeq ($(CONFIG_MACH_MT6771), y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/eem_v2/$(MTK_PLATFORM)/inc/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include/cpuidle_v3/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/cpufreq_v1/src/mach/$(MTK_PLATFORM)/
obj-y += mtk_idle_mt6771.o
obj-y += mtk_spm_dpidle_mt6771.o
obj-y += mtk_spm_sleep_mt6771.o
obj-y += mtk_spm_sodi_mt6771.o
endif
