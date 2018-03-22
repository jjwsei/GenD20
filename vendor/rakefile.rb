PROJECT_CEEDLING_ROOT = "vendor/ceedling"
load "#{PROJECT_CEEDLING_ROOT}/lib/ceedling.rb"

Ceedling.load_project

require 'fileutils'

desc "Build code-genned files for data logging encoder and decoder"
task :codegen_logger do
  sh %{ tools/codegen/bin/codegen_all.sh }
end

task :default => %w[ codegen_logger test:all release ]
