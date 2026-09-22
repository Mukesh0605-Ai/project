from selenium import webdriver
from selenium.webdriver.chrome.options import Options
import time

options = Options()
options.add_argument('--headless')
options.add_argument('--log-level=0')
options.set_capability('goog:loggingPrefs', {'browser': 'ALL'})

driver = webdriver.Chrome(options=options)
driver.get('http://localhost:3000/index.html')
time.sleep(2)

print('INITIAL LOGS')
for entry in driver.get_log('browser'): print(entry)

try:
    btn = driver.find_element('xpath', "//button[@data-id='pragati']")
    driver.execute_script('arguments[0].click();', btn)
    time.sleep(2)
except Exception as e:
    print('Failed to click preset:', e)

print('PRESET LOGS')
for entry in driver.get_log('browser'): print(entry)

try:
    start = driver.find_element('id', 'btn-start-nav')
    driver.execute_script('arguments[0].click();', start)
    time.sleep(2)
except Exception as e:
    print('Failed to click start:', e)

print('START LOGS')
for entry in driver.get_log('browser'): print(entry)

driver.quit()
