require 'ceedling/plugin'
require 'ceedling/constants'
require 'erb'
require 'fileutils'

class ModuleGenerator < Plugin

  attr_reader :config

  def create(module_name, optz={})

    require "generate_module.rb" #From Unity Scripts

    if ((!optz.nil?) && (optz[:destroy]))
      UnityModuleGenerator.new( divine_options(optz) ).destroy(module_name)
    else
      UnityModuleGenerator.new( divine_options(optz) ).generate(module_name)
    end
  end

  private

  def divine_options(optz={})
    {
      :path_src     => ((defined? MODULE_GENERATOR_SOURCE_ROOT ) ? MODULE_GENERATOR_SOURCE_ROOT.gsub('\\', '/').sub(/^\//, '').sub(/\/$/, '') : "src" ),
      :path_inc     => ((defined? MODULE_GENERATOR_SOURCE_ROOT ) ? MODULE_GENERATOR_SOURCE_ROOT.gsub('\\', '/').sub(/^\//, '').sub(/\/$/, '') : "src" ),
      :path_tst     => ((defined? MODULE_GENERATOR_TEST_ROOT   ) ? MODULE_GENERATOR_TEST_ROOT.gsub(  '\\', '/').sub(/^\//, '').sub(/\/$/, '') : "test" ),
      :pattern      => optz[:pattern],
      :test_prefix  => ((defined? PROJECT_TEST_FILE_PREFIX     ) ? PROJECT_TEST_FILE_PREFIX : "Test" ),
      :mock_prefix  => ((defined? CMOCK_MOCK_PREFIX            ) ? CMOCK_MOCK_PREFIX : "Mock" ),
      :includes     => ((defined? MODULE_GENERATOR_INCLUDES    ) ? MODULE_GENERATOR_INCLUDES : {} ),
      :boilerplates => ((defined? MODULE_GENERATOR_BOILERPLATES) ? MODULE_GENERATOR_BOILERPLATES : {} ),
      :naming       => ((defined? MODULE_GENERATOR_NAMING      ) ? MODULE_GENERATOR_NAMING : nil ),
      :update_svn   => ((defined? MODULE_GENERATOR_UPDATE_SVN  ) ? MODULE_GENERATOR_UPDATE_SVN : false ),
    }
  end

end
