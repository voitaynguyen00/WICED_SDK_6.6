1) Create a Podfile from xcode
2) Copy the below contents without braces

# Start of the Podfile.
source 'https://github.com/CocoaPods/Specs.git'
platform :ios, '10.0'
use_frameworks!

target 'WicedWiFiOnboarding' do
    pod 'Alamofire', '~> 4.4'
end
# End of the Pod file.

3) Open terminal in this folder and type these commands
   a) sudo gem install cocoapods
   b) pod install

4) It will ask to close the Xcode and open the new pod project created.