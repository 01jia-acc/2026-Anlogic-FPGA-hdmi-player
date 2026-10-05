# Invoke from tests/build using td_commands_prompt.exe ../check_td.tcl.
# The vendor encrypted HDMI/SDRAM blocks need the native TD compiler.
set repo_root [file normalize [file join [file dirname [info script]] ..]]
set build_dir [pwd]
cd [file join $repo_root src td_project]
import_device eagle_s20.db -package EG4S20BG256
open_project [file join $repo_root src td_project lab_ex5_i2s_v1.0.al] -single_run
elaborate -top top
read_adc [file join $repo_root src user_source constraints_source pin.adc]
read_sdc [file join $repo_root src user_source constraints_source timing.sdc]
read_sdc -ip rfifo_32_32_512 [file join $repo_root src user_source hdl_source IP afifo_32_16_256.tcl]
read_sdc -ip wfifo_32_32_512 [file join $repo_root src user_source hdl_source IP afifo_16_32_256.tcl]
cd $build_dir
optimize_rtl
report_area -file audio_top_rtl.area
optimize_gate -packarea audio_top_gate.area
puts "PASS: native TD top-level synthesis"
exit
