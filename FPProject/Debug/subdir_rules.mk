################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
COECSL_edma3.obj: C:/ngraves2/repository/FP/src/COECSL_edma3.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="COECSL_edma3.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

COECSL_mcbsp.obj: C:/ngraves2/repository/FP/src/COECSL_mcbsp.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="COECSL_mcbsp.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ColorLCD.obj: C:/ngraves2/repository/FP/src/ColorLCD.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="ColorLCD.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

ColorVision.obj: C:/ngraves2/repository/FP/src/ColorVision.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="ColorVision.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FPcfg.cmd: C:/ngraves2/repository/FP/DSPBIOS/FP.tcf
	@echo 'Building file: $<'
	@echo 'Invoking: TConf'
	"C:/CCStudio_v6.0/bios_5_42_01_09/xdctools/tconf" -b -Dconfig.importPath="C:/CCStudio_v6.0/bios_5_42_01_09/packages;C:/ngraves2/repository/FP/DSPBIOS;" "$<"
	@echo 'Finished building: $<'
	@echo ' '

FPcfg.s??: | FPcfg.cmd
FPcfg_c.c: | FPcfg.cmd
FPcfg.h: | FPcfg.cmd
FPcfg.h??: | FPcfg.cmd
FP.cdb: | FPcfg.cmd

FPcfg.obj: ./FPcfg.s?? $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="FPcfg.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

FPcfg_c.obj: ./FPcfg_c.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="FPcfg_c.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

LCDprintf.obj: C:/ngraves2/repository/FP/src/LCDprintf.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="LCDprintf.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

Ladar.obj: C:/ngraves2/repository/FP/src/Ladar.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="Ladar.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

MatrixMath.obj: C:/ngraves2/repository/FP/src/MatrixMath.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="MatrixMath.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

pru.obj: C:/ngraves2/repository/FP/src/pru.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="pru.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

user_FP.obj: C:/ngraves2/repository/FP/src/user_FP.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="user_FP.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

user_xy.obj: C:/ngraves2/repository/FP/src/user_xy.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C6000 Compiler'
	"C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/bin/cl6x" -mv6740 --abi=coffabi -O3 -g --include_path="C:/CCStudio_v6.0/ccsv6/tools/compiler/c6000_7.4.8/include" --include_path="C:/ngraves2/repository/bsl/inc" --include_path="C:/ngraves2/repository/FP" --include_path="C:/ngraves2/repository/c67xmathlib_2_01_00_00/inc" --include_path="C:/ngraves2/repository/mcbsp_com" --include_path="C:/ngraves2/repository/sharedmem_com" --include_path="C:/ngraves2/repository/FP/include" --include_path="C:/ngraves2/repository/FP/FPProject/Debug" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/bios/include" --include_path="C:/CCStudio_v6.0/bios_5_42_01_09/packages/ti/rtdx/include/c6000" --define=RUNNING_ON_OMAPL138 --define=_DEBUG --display_error_number --diag_warning=225 --preproc_with_compile --preproc_dependency="user_xy.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


