file(REMOVE_RECURSE
  "librevolve-spec.pdb"
  "librevolve-spec.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/revolve-spec.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
