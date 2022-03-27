#! /usr/bin/env python
import datetime
import math
import sys
import os
import time
import subprocess
import random
import csv
import serial
import json
# import keyboard
import logging
from numpy import interp
from threading import Thread
from distutils.spawn import find_executable
from MercuryController import *


class MercurySim:
    def __init__(self):

        self.config = json.loads(open('config.json').read())

        # Mission Time
        self.running = False
        self.launch_time = None
        self.mission_time = None

        self.running_thread = None

        # Scenes
        self.current_scene = 0
        self.scenes = list(csv.DictReader(open('scenes.csv')))

        # Mission Events
        self.mission_events = list(csv.DictReader(open('events_launch.csv')))
        self.retrograde_time = t_time_to_seconds([x for x in self.mission_events if x['name'] == "retrofire"][0]['time'])
        self.event_history = []

        # Window
        # self.window = WindowDisplay()

        # Displays
        self.time_display = TimeDisplay()
        self.time_display.set_retrograde_time(self.retrograde_time)

        # Audio Devices
        self.sfx_audio = AudioController("sfx", output=0, volume='100%')
        self.capcom_audio = AudioController("capcom", output=1, volume='40%')

        # LEDs
        self.leds = {}
        for x in csv.DictReader(open('led_mapping.csv')):
            if x['name']:
                self.leds[x['name']] = x['number']

        # Servos
        self.servos = {}
        for x in csv.DictReader(open('servo_mapping.csv')):
            if x['name']:
                self.servos[x['name']] = x

        # Draw default stars
        subprocess.Popen('fbi -a stars.jpg -d /dev/fb0 -T 1 --noverbose', shell=True)

        try:
            self.controller = MercuryController(None, 9600)
        except Exception as e:
            logging.error("Unable to connect to Mercury Arduino: {}".format(e))
            self.controller = None

    def teardown(self):
        logging.info('')
        logging.info('{:-^40}'.format(' Teardown '))

        AudioController.stop_all_audio()
        subprocess.Popen(['sudo', 'killall', 'omxplayer.bin'])  # stop video


    # -- Mission Clock

    def update_time(self):
        time = time_from(self.launch_time)
        updated_time = self.mission_time != time
        self.mission_time = time
        return updated_time

    # -- Servos

    def set_servo_value(self, val, servo):
        x = interp(val,[float(servo['minval']),float(servo['maxval'])],[float(servo['minpercent']),float(servo['maxpercent'])])
        logging.debug("Set servo {} value to {}".format(servo['name'], x))
        if self.controller:
            self.controller.set_servo_percent(servo['number'], x)

    # -- Audio

    def play_timed_audio(self):

        # Play timed capcom files
        if self.mission_time < 1:
            timed_capcom = "capcom_{}.wav".format(abs(self.mission_time))
        else:
            timed_capcom = "capcom_p{}.wav".format(abs(self.mission_time))
        if timed_capcom in self.capcom_audio.files:
            self.capcom_audio.play_file(timed_capcom)

    def update_time_display(self):
        # Update time display
        time_to_retrograde = max(self.retrograde_time - max(self.mission_time, 0), 0)
        self.time_display.set_time_to_retrograde(time_to_retrograde)
        self.time_display.set_time_from_launch(max(self.mission_time, 0))

    def trigger_event(self, event_name, allow_failure=False):

        # Don't trigger event if already done
        if event_name in self.event_history:
            logging.error("{} already triggered".format(event_name))
            return

        # Trigger videos first
        if 'video' in event_name:
            video_file = [x for x in os.listdir('video') if event_name.split('-')[-1] in x]
            if video_file:
                logging.info("Playing video file: {}".format(video_file))
                play_video(video_file[0])

        good_led = self.leds.get(event_name, None) or self.leds.get(event_name + "_green", None)
        succeed = True if not allow_failure else not (random.randint(1, 10) == 5 and bool(good_led))

        if succeed:
            logging.info("Mission Event: " + event_name)
            sfx_file = self.sfx_audio.file_with_name(event_name)
            capcom_file = self.capcom_audio.file_with_name(event_name)

            if sfx_file:
                self.sfx_audio.play_file(sfx_file)
            if capcom_file:
                self.capcom_audio.play_file(capcom_file)
            if good_led and self.controller:
                self.controller.led_on(good_led)
            self.event_history.append(event_name)
        else:
            logging.info("Mission Event (Failure): " + event_name)
            bad_led = self.leds.get(event_name + "_red", None)
            if bad_led and self.controller:
                self.controller.led_on(bad_led)

    def key_press(self, event):

        key = event.name

        if key in ['a', 'b', 'c', 'd']:

            logging.info('Pressed {} key'.format(key))
            self.running = False
            if self.running_thread:
                logging.info('-------Stopping thread-------')
                self.running_thread.join()
            if key == 'a':  # start sim
                self.running_thread = Thread(target=self.run_simulation)
            else:  # start scene
                idx = ord(key) - 98  # convert char to int (i.e. b into 0, c into 1, etc)
                self.running_thread = Thread(target=self.run_scene, args=(idx,))
            self.running_thread.daemon = True
            self.running_thread.start()
        # elif key is not None:
        #     logging.error('Unknown key!')

    def run_scene(self, scene_number):

        logging.info('')

        logging.info('{:-^40}'.format(' Starting Scene #{} '.format(scene_number)))

        scene = self.scenes[scene_number]
        play_video(scene['video'], loop=True, random_orientation=True)
        # self.sfx_audio.play_file(scene['audio'], loop=True)

        # Play ambient audio
        self.sfx_audio.play_file("sfx_cabin_loop.wav", loop=True)

        next_misc_capcom = 0

        self.running = True
        while self.running:
            if not next_misc_capcom:
                capcom_timings = self.config['capcom'][scene['mission_phase']]
                next_misc_capcom = random.randint(capcom_timings['min'], capcom_timings['max'])
                logging.info('next capcom in {}s'.format(t_time(next_misc_capcom)))

            time.sleep(1)

            next_misc_capcom -= 1
            if not next_misc_capcom:
                self.capcom_audio.play_random_file(scene['mission_phase'])
        self.teardown()

    def get_acceleration(self):

        # Find Mission Events
        old_events = [x for x in self.mission_events if t_time_to_seconds(x.get("time")) < self.mission_time and x.get('acceleration')]
        next_events = [x for x in self.mission_events if t_time_to_seconds(x.get("time")) >= self.mission_time and x.get('acceleration')]

        if old_events and next_events:
            last_event = old_events[-1]
            next_event = next_events[0]
            if last_event['acceleration'] == next_event['acceleration']:
                return float(next_event['acceleration'])
            else:
                x = interpolated_value(start_time=t_time_to_seconds(last_event["time"]), start_val=last_event['acceleration'],
                                       end_time=t_time_to_seconds(next_event['time']), end_val=next_event['acceleration'],
                                       current_time=self.mission_time, rate=2)
                return x
        elif old_events:
            return float(old_events[-1].get('acceleration'))
        elif next_events:
            return float(next_events[0].get('acceleration'))
        else:
            logging.error('Get acceleration - Cannot find any events!')

    def run_simulation(self):

        logging.info('')
        logging.info('{:-^40}'.format(' Starting Sim '))

        # Setup launch time
        self.launch_time = datetime.datetime.now() + datetime.timedelta(seconds=(self.config['prelaunch_time'] + 1))
        orbit_time = [x for x in self.mission_events if 'orbit_phase' in x['name']][0]['time']

        # Clear history
        del self.event_history[:]
        self.capcom_audio.clear_history()
        self.sfx_audio.clear_history()

        # Play video
        play_video('launch.mp4', loop=False)

        # Play ambient audio
        self.sfx_audio.play_file("sfx_cabin_loop.wav", loop=True)

        self.running = True
        next_misc_capcom = None
        while self.running:

            # Don't kill CPU
            time.sleep(0.1)

            # Look for button updates
            # self.controller.has_updates()

            # Trigger automatic functions
            if self.update_time():

                # Mission Time
                current_t_time = t_time(self.mission_time)
                acceleration = self.get_acceleration()

                logging.info("{} - Accel: {:.1f}g".format(current_t_time, acceleration))
                self.play_timed_audio()
                self.update_time_display()

                # Update Servos
                self.set_servo_value(acceleration, self.servos['acceleration'])

                # logging.info("Acceleration - {}".format(self.get_acceleration()))

                # Trigger mission events
                now_events = [x for x in self.mission_events if x.get("time") == current_t_time]
                for event in now_events:
                    self.trigger_event(event['name'], allow_failure=self.config['allow_failures'])

                if self.mission_time < 0:
                    mission_phase = 'prelaunch'
                elif self.mission_time < t_time_to_seconds(orbit_time):
                    mission_phase = 'launch'
                else:
                    mission_phase = 'orbit'

                # Play Capcom Random
                if next_misc_capcom == self.mission_time:

                    self.capcom_audio.play_random_file(mission_phase)
                    next_misc_capcom = None

                # Setup next random capcom
                if not next_misc_capcom:
                    capcom_timings = self.config['capcom'][mission_phase]
                    next_misc_capcom = random.randint(capcom_timings['min'], capcom_timings['max']) + self.mission_time
                    while True:
                        next_ttime = t_time(next_misc_capcom)
                        next_ttime_plus = t_time(next_misc_capcom + 1)  # 1s padding
                        collision = any(next_ttime in x['time'] or next_ttime_plus in x['time'] for x in self.mission_events)
                        timed_format = 'capcom_{}'.format(abs(next_misc_capcom)) if next_misc_capcom < 0 else 'capcom_p{}'.format(abs(next_misc_capcom))
                        collision = collision or any(timed_format in x for x in self.capcom_audio.files)
                        if collision:
                            # logging.info('possible capcom collision. adding 2')
                            # logging.info('was: {}'.format(next_misc_capcom))
                            next_misc_capcom += 2
                            # logging.info('now: {}'.format(next_misc_capcom))
                        else:
                            break

                    logging.info('Next misc capcom at {}'.format(next_misc_capcom))

        self.teardown()

class AudioController:
    def __init__(self, type, output, volume):
        self.player = find_executable('afplay') or find_executable('aplay')
        self.type = type
        self.play_history = []
        self.files = [f for f in os.listdir('audio') if self.type in f]
        self.output = str(output)

        # Set volume to 100% on Pi
        if "aplay" in self.player:
            subprocess.call(['amixer', '-c', self.output, 'set', 'PCM', '--', volume])
            logging.info('Setting volume on output {0} to {1}'.format(self.output, volume))

    def play_random_file(self, prefix):
        prefix = '_' + prefix + '_misc'
        misc_audio = [f for f in self.files if prefix in f]
        unplayed_audio = [m for m in misc_audio if m not in self.play_history]
        if not unplayed_audio:
            logging.info('Used all available audio. Resetting history')
            self.clear_history()
            unplayed_audio = misc_audio
        while True:
            misc_file = random.choice(unplayed_audio)
            if misc_file not in self.play_history:
                break
        self.play_file(misc_file)

    def file_with_name(self, name):
        found_file = None
        files = [f for f in self.files if os.path.splitext(f)[0].endswith(name)]
        if files:
            found_file = files[0]
        return found_file

    def play_file(self, filename, loop=False):
        logging.info("Playing {0} audio file: {1}".format(self.type, filename))
        dir_path = os.path.dirname(os.path.realpath(__file__))
        filepath = os.path.join(dir_path, "audio", filename)

        if self.output and "aplay" in self.player:
            if loop:
                # only omxplayer can loop
                command = ['omxplayer', '--no-keys', '--no-osd', '--loop', filepath]
            else:
                # aplay for everything else
                command = [self.player, '-D', 'default:{}'.format(self.output), '-c', '1', filepath]
        else:
            # macOS
            command = [self.player, filepath]

        subprocess.Popen(command)
        self.play_history.append(filename)

    def clear_history(self):
        del self.play_history[:]

    @classmethod
    def stop_all_audio(cls):
        logging.info("Stopping all audio")
        player = find_executable('afplay') or find_executable('aplay')
        subprocess.call(["killall", os.path.basename(player)])


class TimeDisplay:

    def __init__(self):
        try:
            self.arduino = serial.Serial('COM1', 115200, timeout=.1)
        except Exception as e:
            logging.error("Cannot connect to TimeDisplay: {}".format(e))

    def set_time_from_launch(self, time):
        if hasattr(self, "arduino"):
            self.arduino.write("time_from {}".format(time))

    def set_time_to_retrograde(self, time):
        if hasattr(self, "arduino"):
            self.arduino.write("time_to {}".format(time))

    def set_retrograde_time(self, time):
        if hasattr(self, "arduino"):
            self.arduino.write("retrograde {}".format(time))


def play_video(filename, loop=False, random_orientation=False, random_start=False):
    player = find_executable('omxplayer')
    if player:
        logging.info("Playing video file: {}".format(filename))
        filepath = os.path.join("video", filename)

        commands = ['sudo', player, '--no-keys', '--no-osd',  filepath, '--layer', '10']
        # else:
        #     commands = ['sudo', player, '--no-keys', '--no-osd', filepath, '--layer', '10']

        if loop:
            commands.append('--loop')

        if random_start:
            commands.append('-p')
            commands.append(random.randint(0, 60))

        if random_orientation:
            commands.append('--orientation')
            commands.append(random.choice(['0', '180']))

        p = subprocess.Popen(commands, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        time.sleep(0.2)  # wait for video to start to not conflict with audio
    else:
        logging.error('No video player found')


# -- Functions --


def t_time_to_seconds(t_time_val):
    raw_time = t_time_val[2:].split(":")
    seconds = time_to_seconds(raw_time[0], raw_time[1], raw_time[2])
    if t_time_val[:2] == "T-":
        seconds = -seconds
    return seconds


def time_to_seconds(hours=0, minutes=0, seconds=0):
    # Define the constants
    SECONDS_PER_MINUTE = 60
    SECONDS_PER_HOUR = 3600
    total_seconds = int(hours) * SECONDS_PER_HOUR
    total_seconds = total_seconds + (int(minutes) * SECONDS_PER_MINUTE)
    total_seconds = total_seconds + int(seconds)

    return total_seconds


def time_from(from_time):
    now = datetime.datetime.now()
    return int((now - from_time).total_seconds())


def formatted_time(time_val):
    time_val = abs(time_val)
    hour = time_val // 3600
    time_val %= 3600
    minutes = time_val // 60
    time_val %= 60
    seconds = time_val

    return "%02d:%02d:%02d" % (hour, minutes, seconds)


def t_time(time):
    prefix = "T-" if time < 0 else "T+"
    t_time = prefix + formatted_time(time)
    return t_time

def interpolated_value(start_time, start_val, end_time, end_val, current_time, rate):

    multiplier = math.pow(current_time-start_time, rate) / math.pow(end_time-start_time, rate)
    current_val = (float(end_val)-float(start_val)) * multiplier + float(start_val)

    return current_val


if __name__ == '__main__':
    format = "%(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO)

    # set cwd
    os.chdir(os.path.dirname(os.path.abspath(__file__)))

    try:
        sim = MercurySim()
        # keyboard.on_press(sim.key_press)
        sim.run_simulation()
        logging.info('Ready')
        # play_video('launch.mp4')
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logging.info("User Interrupt")
        subprocess.Popen(['sudo', 'killall', 'fbi'])
        sim.running = False
        sim.teardown()
        sys.exit(0)
