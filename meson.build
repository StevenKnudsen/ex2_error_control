project('ex2_obc_sdr', 'c', 'cpp',
    license: 'proprietary',
    version: '0.5.0',
    default_options : [
        'buildtype=release',
        'warning_level=3',
#        'buildtype=debug',
#        'werror=true',
        ],
    )

cpp = meson.get_compiler('cpp')

home = run_command('sh', '-c', 'echo $HOME', check: true)
HOME_DIR = home.stdout().strip()
message('home = ' + HOME_DIR)

prj_dir = run_command('sh', '-c', 'echo $PWD', check: true)
PRJ_DIR = prj_dir.stdout().strip()
message('prj_dir = ' + PRJ_DIR)

#eigen_dep = dependency('eigen3')

gtest_dep = dependency('gtest_main', required: false)

if not gtest_dep.found()
    foreach incdir: ['/usr/include/gtest', '/usr/local/include/gtest']
        if cpp.has_header('gtest.h', args: '-I' + incdir)
            gtest_inc = include_directories(incdir)
            break
        endif
    endforeach
    
    gtest_dep = declare_dependency(
        dependencies: [cpp.find_library('gtest'),cpp.find_library('gtest_main')],
        include_directories: gtest_inc,
        )
endif

if not gtest_dep.found()
    error('unable to find gtest dependency')
endif


core_source_files = [
    PRJ_DIR / 'src/convolutional_codec_hd.cpp',
    PRJ_DIR / 'src/error_correction.cpp',
    PRJ_DIR / 'src/fec.cpp',
    PRJ_DIR / 'src/golay.cpp',
    PRJ_DIR / 'src/no_fec.cpp',
    PRJ_DIR / 'src/qcldpc.cpp'
]

incdir = include_directories(
    'include',
    'include/third_party/viterbi'
)

third_party_source_files = [
    PRJ_DIR / 'src/third_party/viterbi/viterbi.cpp'
]

if gtest_dep.found()
    subdir('unit_tests')
endif

subdir('doc')
