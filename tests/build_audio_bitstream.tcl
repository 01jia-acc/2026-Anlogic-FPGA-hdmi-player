# Run with td_commands_prompt.exe tests/build_audio_bitstream.tcl.
# Keep reports in the release directory and synchronize GUI download bitstreams.
set repo_root [file normalize [file join [file dirname [info script]] ..]]
set build_dir [file join $repo_root tests build pcm_audio_release]
file mkdir $build_dir
set project_path [file join $repo_root src td_project lab_ex5_i2s_v1.0.al]
set project_handle [open $project_path r]
set project_text [read $project_handle]
close $project_handle
foreach source {audio/pcm_stream_buffer.v audio/pcm_test_tone.v audio/pcm_media_tone.v audio/media_hdmi_audio_bridge.v SD/media_display_commit.v SD/sd_card_bmp_fat32.v SD/media_frame_loader.v SD/media_frame_reader.v SD/media_sdram_write_cdc.v SD/promax_audio_waveform.v} {
    if {[string first "$source" $project_text] < 0} {
        error "Missing integration source in project: $source. Reload the updated .al in TD."
    }
}
cd [file join $repo_root src td_project]
foreach source {SD/scrolling_subtitle.v SD/subtitle_font.v SD/audio4_subtitle_data.v SD/audio4_subtitle_timeline.v} {
    if {[string first "$source" $project_text] < 0} {
        error "Missing subtitle integration source in project: $source. Reload the updated .al in TD."
    }
}
import_device eagle_s20.db -package EG4S20BG256
open_project [file join $repo_root src td_project lab_ex5_i2s_v1.0.al] -single_run
elaborate -top top
read_adc [file join $repo_root src user_source constraints_source pin.adc]
read_sdc [file join $repo_root src user_source constraints_source timing.sdc]
read_sdc -ip rfifo_32_32_512 [file join $repo_root src user_source hdl_source IP afifo_32_16_256.tcl]
read_sdc -ip wfifo_32_32_512 [file join $repo_root src user_source hdl_source IP afifo_16_32_256.tcl]
cd $build_dir
optimize_rtl
report_area -file hdmi_four_buffer_audio_rtl.area
optimize_gate -packarea hdmi_four_buffer_audio_gate.area
legalize_phy_inst
update_timing
report_timing_status -file hdmi_four_buffer_audio_gate.ts
report_timing_summary -file hdmi_four_buffer_audio_gate.timing
export_db hdmi_four_buffer_audio_gate.db
place
update_timing -mode manhattan
report_timing_summary -file hdmi_four_buffer_audio_place.timing
report_area -io_info -file hdmi_four_buffer_audio_phy.area
export_db hdmi_four_buffer_audio_place.db
route
report_area -io_info -file hdmi_four_buffer_audio_phy.area
update_timing -mode final
report_timing_status -file hdmi_four_buffer_audio_phy.ts
report_timing_summary -file hdmi_four_buffer_audio_pr.timing
report_timing_exception -file hdmi_four_buffer_audio_exception.timing
export_db hdmi_four_buffer_audio_pr.db
bitgen -bit hdmi_four_buffer_audio.bit
source [file join $repo_root tests sync_audio_bitstream.tcl]
puts "PASS: built $build_dir/hdmi_four_buffer_audio.bit; review final timing before hardware testing"
exit
