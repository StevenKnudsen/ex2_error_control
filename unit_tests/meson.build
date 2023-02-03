#
# TODO Could split out only those source files needed to support each test. See
# golay test for example.

incdirUT = include_directories(
    '../include/'
)

unit_test_golay = executable('unit_test-golay', 'qa_golay.cpp', '../src/golay.cpp',
    include_directories : incdirUT,
    dependencies: [gtest_dep]
    )
    
test('golay', unit_test_golay,
    timeout: 30
    )
   
unit_test_No_FEC = executable('unit_test-No_FEC', 'qa_no_fec.cpp', core_source_files, third_party_source_files,
    include_directories : incdirUT,
    dependencies: [gtest_dep]
    )
    
test('NoFEC', unit_test_No_FEC,
    timeout: 30
    )

unit_test_viterbi = executable('unit_test-viterbi', 'qa_viterbi.cpp', core_source_files, third_party_source_files,
    include_directories : incdirUT,
    dependencies: [gtest_dep]
    )
    
test('viterbi', unit_test_viterbi,
    timeout: 100
    )

#unit_test_CC_HD_FEC = executable('unit_test-CC_HD_FEC', 'qa_convolutional_codec_hd.cpp', 'vectorTools.cpp', core_source_files, third_party_source_files,
#    include_directories : incdirUT,
#    dependencies: [gtest_dep]
#    )
#   
#test('CC_HD_FEC', unit_test_CC_HD_FEC,
#    timeout: 100
#    )

