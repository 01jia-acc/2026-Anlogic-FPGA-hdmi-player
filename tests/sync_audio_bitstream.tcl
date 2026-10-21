# Executed after a successful build, or sourced to synchronize an existing release.
set sync_repo_root [file normalize [file join [file dirname [info script]] ..]]
set sync_build_dir [file join $sync_repo_root tests build pcm_audio_release]
set sync_release [file join $sync_build_dir hdmi_four_buffer_audio.bit]
if {![file exists $sync_release] || [file size $sync_release] == 0} {
    error "Missing audio release bitstream: $sync_release"
}
file copy -force $sync_release [file join $sync_build_dir hdmi_pcm_loop.bit]
file copy -force $sync_release [file join $sync_build_dir hdmi_multi_bmp_audio.bit]
file copy -force $sync_release [file join $sync_build_dir hdmi_prefetch_audio.bit]
file copy -force $sync_release [file join $sync_build_dir hdmi_prefetch_pipeline_audio.bit]
file copy -force $sync_release [file join $sync_build_dir hdmi_image_pcm_audio.bit]
file copy -force $sync_release [file join $sync_build_dir hdmi_audio_waveform.bit]
foreach sync_run {phy_1 best_result} {
    set sync_target [file join $sync_repo_root src td_project lab_ex5_i2s_v1.0_Runs $sync_run lab_ex5_i2s_v1.0.bit]
    if {[file exists $sync_target]} {
        set sync_backup_dir [file join $sync_build_dir previous_gui_bitstreams]
        file mkdir $sync_backup_dir
        set sync_backup [file join $sync_backup_dir ${sync_run}_[file mtime $sync_target].bit]
        if {![file exists $sync_backup]} {
            file copy $sync_target $sync_backup
        }
        file copy -force $sync_release $sync_target
        puts "Synchronized download bitstream: $sync_target"
    }
}
puts "Per-image PCM download: [file join $sync_build_dir hdmi_image_pcm_audio.bit]"
