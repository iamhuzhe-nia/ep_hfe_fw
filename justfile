# use cmd.exe instead of sh:
set shell := ["cmd.exe", "/c"]


uf2:
  elf2uf2-rs target/thumbv6m-none-eabi/debug/ep-hfe


upload target: uf2
  copy target\\thumbv6m-none-eabi\\debug\\ep-hfe.uf2 {{target}}