#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 20/10/15

@author: Daniel deCordoba

This node opens a Chrome browser, subscribes to the "/web/go_to"
topic and manages the browser depending on the info received there
"""
import os
import shutil
import rospy
import rospkg
import socket
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.chrome.options import Options
from selenium.common.exceptions import TimeoutException
from selenium.common.exceptions import WebDriverException
from selenium.webdriver.chrome.service import Service
import pal_web_msgs.msg as web
import diagnostic_msgs.msg as diag_msgs


def copy_file(src, dest, name):
    if not os.path.exists(dest):
        os.makedirs(dest)
    if os.path.isfile(dest+name):
        os.remove(dest+name)
    shutil.copy(src+name, dest)
    rospy.loginfo("File " + src+name + " copied to " + dest)

class browserLoader:

    def __init__(self):
        #Get path source files to copy
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('pal_chrome')
        #Copy Preferences Chrome to our preferences
        copy_file(pkgpath+"/config", "/tmp/chrome-profile/Default", "/Preferences")
        copy_file(pkgpath+"/config", "/tmp/chrome-profile", "/Local State")
        #Create Chrome Options
        self.chrome_options = Options()
        self.chrome_options.add_argument("--user-data-dir=/tmp/chrome-profile")
        self.chrome_options.add_argument("--app=http://control:8080")
        self.chrome_options.add_argument("--kiosk")
        self.chrome_options.add_argument("--disable-application-cache")
        self.chrome_options.add_argument("--media-cache-size=1")
        self.chrome_options.add_argument("--disk-cache-size=1")
        self.chrome_options.add_argument("--disable-features=TranslateUI")
        self.chrome_options.add_argument("--disable-features=Translate")
        self.chrome_options.add_argument("--disable-features=kTranslate")
        self.chrome_options.add_argument("--disable-translate")
        self.chrome_options.add_argument("--disable-pinch")
        self.chrome_options.add_argument("--use-fake-ui-for-media-stream")
        self.chrome_options.add_argument("--disable-features=OverscrollHistoryNavigation")
        self.chrome_options.add_argument("--autoplay-policy=no-user-gesture-required")
        # Dont show infobar "Chrome is being controlled..."
        self.chrome_options.add_argument('--disable-infobars')
        self.chrome_options.add_experimental_option("excludeSwitches", ["enable-automation"])
        #Start Browser
        self.path_chromedriver = '/opt/chromedriver/chromedriver'
        #try:
        #    self.browser = webdriver.Chrome(executable_path=self.path_chromedriver, chrome_options=self.chrome_options)
        #except:
        self.service = Service(self.path_chromedriver)
        self.browser = webdriver.Chrome(service=self.service,options=self.chrome_options)
        #Init browseR to white page
        # self.browser.get("about:blank")
        socket.setdefaulttimeout(15)
        rospy.loginfo("Chrome Browser started")

    def setUrl(self, url):
        try:
            rospy.loginfo("Navigating to url " + url)
            self.browser.get(url)
        except TimeoutException as ex:
            rospy.logwarn(ex.Message)
        except WebDriverException as e:
            rospy.logwarn(e)
            rospy.loginfo("Memory probably overloaded, next attempt in 5 s")
            rospy.sleep(5)
            self.browser = webdriver.Chrome(executable_path=self.path_chromedriver, chrome_options=self.chrome_options)
            self.browser.get(url)
        #search = 'pal robotics'
        #elem = self.browser.find_element_by_name('p')  # Find the search box
        #elem.send_keys(search + Keys.RETURN)

    def getUrl(self):
        return self.browser.current_url

    def openBrowser(self):
        self.browser = webdriver.Chrome(executable_path=self.path_chromedriver, chrome_options=self.chrome_options)

    def closeBrowser(self):
        self.browser.quit()

    def forward(self):
        self.browser.forward()

    def back(self):
        self.browser.back()

    def refresh(self):
        self.browser.refresh()

    def getconfig(self, index): #Index => 0: keys, 1: values, 2: items
        #Get array with browser params
        if index == 1:
            return self.browser.capabilities.keys()
        elif index == 2: #items = [(key0,value0), (key1,value1), ...]
            return self.browser.capabilities.items()
        else:
            return self.browser.capabilities.values()

    def __del__(self):
 	    pass

class main(object):
    def __init__(self):
        rospy.init_node('pal_chrome')
        self.chrome = browserLoader()
        rospy.Subscriber("/web/go_to", web.WebGoTo, self.callback)
        self.diag_pub = rospy.Publisher("diagnostics", diag_msgs.DiagnosticArray, queue_size=1)
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        rospy.spin()

    def callback(self, data):
        typ = data.type
        value = data.value
        if typ == 0: #Open image
            self.chrome.setUrl("file://{}".format(value))
        elif typ == 1: #Open video
            self.chrome.setUrl("file://{}".format(value))
        elif typ == 2: #Go to URL
            self.chrome.setUrl(value)
        elif typ == 3: #Go to Webapp
            self.chrome.setUrl("http://localhost" + value)
        else:
            rospy.loginfo("Unknown type {} with value {}".format(typ, value))

    def publish_diagnostics(self, ev):
        array = diag_msgs.DiagnosticArray()
        status = diag_msgs.DiagnosticStatus()

        status.name = "Functionality: Chrome"
        array.status.append(status)

        self.diag_pub.publish(array)

if __name__ == '__main__':
    main()

