# Copyright 2015 Yale University - Grablab
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:\
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import sys
import json
import urllib.request
import urllib.error
import tarfile

output_directory = "./ycb"

# You can either set this to "all" or a list of the objects that you'd like to download.
objects_to_download = [
    "001_chips_can",
    "002_master_chef_can",
    "003_cracker_box",
    "004_sugar_box",
    "005_tomato_soup_can",
    "006_mustard_bottle",
    "007_tuna_fish_can",
    "008_pudding_box",
    "009_gelatin_box",
    "010_potted_meat_can",
    "011_banana",
    "012_strawberry",
    "013_apple",
    "014_lemon",
    "015_peach",
    "016_pear",
    "017_orange",
    "018_plum",
    "019_pitcher_base",
    "021_bleach_cleanser",
    "022_windex_bottle",
    "023_wine_glass",
    "024_bowl",
    "025_mug",
    "026_sponge",
    "027-skillet",
    "028_skillet_lid",
    "029_plate",
    "030_fork",
    "031_spoon",
    "032_knife",
    "033_spatula",
    "035_power_drill",
    "036_wood_block",
    "037_scissors",
    "038_padlock",
    "039_key",
    "040_large_marker",
    "041_small_marker",
    "042_adjustable_wrench",
    "043_phillips_screwdriver",
    "044_flat_screwdriver",
    "046_plastic_bolt",
    "047_plastic_nut",
    "048_hammer",
    "049_small_clamp",
    "050_medium_clamp",
    "051_large_clamp",
    "052_extra_large_clamp",
    "053_mini_soccer_ball",
    "054_softball",
    "055_baseball",
    "056_tennis_ball",
    "057_racquetball",
    "058_golf_ball",
    "059_chain",
    "061_foam_brick",
    "062_dice",
    "063-a_marbles",
    "063-b_marbles",
    "063-c_marbles",
    "063-d_marbles",
    "063-e_marbles",
    "063-f_marbles",
    "065-a_cups",
    "065-b_cups",
    "065-c_cups",
    "065-d_cups",
    "065-e_cups",
    "065-f_cups",
    "065-g_cups",
    "065-h_cups",
    "065-i_cups",
    "065-j_cups",
    "070-a_colored_wood_blocks",
    "070-b_colored_wood_blocks",
    "071_nine_hole_peg_test",
    "072-a_toy_airplane",
    "072-b_toy_airplane",
    "072-c_toy_airplane",
    "072-d_toy_airplane",
    "072-e_toy_airplane",
    "072-f_toy_airplane",
    "072-g_toy_airplane",
    "072-h_toy_airplane",
    "072-i_toy_airplane",
    "072-j_toy_airplane",
    "072-k_toy_airplane",
    "073-a_lego_duplo",
    "073-b_lego_duplo",
    "073-c_lego_duplo",
    "073-d_lego_duplo",
    "073-e_lego_duplo",
    "073-f_lego_duplo",
    "073-g_lego_duplo",
    "073-h_lego_duplo",
    "073-i_lego_duplo",
    "073-j_lego_duplo",
    "073-k_lego_duplo",
    "073-l_lego_duplo",
    "073-m_lego_duplo",
    "076_timer",
    "077_rubiks_cube"
  ]
# objects_to_download = ["all"]

# You can edit this list to only download certain kinds of files.
# 'berkeley_rgbd' contains all of the depth maps and images from the Carmines.
# 'berkeley_rgb_highres' contains all of the high-res images from the Canon cameras.
# 'berkeley_processed' contains all of the segmented point clouds and textured meshes.
# 'google_16k' contains google meshes with 16k vertices.
# 'google_64k' contains google meshes with 64k vertices.
# 'google_512k' contains google meshes with 512k vertices.
# See the website for more details.
# files_to_download = ["berkeley_rgbd", "berkeley_rgb_highres", "berkeley_processed", "google_16k", "google_64k", "google_512k"]
files_to_download = ["berkeley_rgb_highres", "berkeley_rgbd"]

# Extract all files from the downloaded .tgz, and remove .tgz files.
# If false, will just download all .tgz files to output_directory
extract = True

# Use HTTPS (stabiler in vielen Notebook-Umgebungen)
base_url = "http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/"
objects_url = base_url + "objects.json"

if not os.path.exists(output_directory):
    os.makedirs(output_directory)

def fetch_objects(url):
    response = urllib.request.urlopen(urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"}))
    html = response.read().decode("utf-8")
    objects = json.loads(html)
    return objects["objects"]

def download_file(url, filename):
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    u = urllib.request.urlopen(req)
    with open(filename, 'wb') as f:
        meta = u.info()
        size_header = meta.get("Content-Length")
        file_size = int(size_header) if size_header is not None else None
        if file_size:
            print(f"Downloading: {filename} ({file_size/1000000.0:.2f} MB)")
        else:
            print(f"Downloading: {filename}")

        file_size_dl = 0
        block_sz = 65536
        while True:
            buffer = u.read(block_sz)
            if not buffer:
                break
            file_size_dl += len(buffer)
            f.write(buffer)
            if file_size:
                pct = file_size_dl * 100.0 / file_size
                status = r"%10.2f MB  [%3.2f%%]" % (file_size_dl/1000000.0, pct)
                print("\r" + status, end="")
        print()

def tgz_url(object, type):
    if type in ["berkeley_rgbd", "berkeley_rgb_highres"]:
        return base_url + "berkeley/{object}/{object}_{type}.tgz".format(object=object,type=type)
    elif type in ["berkeley_processed"]:
        return base_url + "berkeley/{object}/{object}_berkeley_meshes.tgz".format(object=object,type=type)
    else:
        return base_url + "google/{object}_{type}.tgz".format(object=object,type=type)

def extract_tgz(filename, dir):
    # gleiche Funktionalität, aber sicher per tarfile
    with tarfile.open(filename, "r:gz") as tar:
        tar.extractall(path=dir)
    os.remove(filename)

def check_url(url):
    # HEAD ist auf S3-Website-Endpoints unzuverlässig; GET versuchen und Fehler auswerten
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req) as _:
            return True
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return False
        return False
    except urllib.error.URLError:
        return False

if __name__ == "__main__":

    # Bewahrt die ursprüngliche Logik, funktioniert aber auch mit "all"
    if objects_to_download == "all":
        objects = fetch_objects(objects_url)
    else:
        objects = objects_to_download

    for object in objects:
        if objects_to_download == "all" or object in objects_to_download:
            for file_type in files_to_download:
                try:
                    print("test")
                    url = tgz_url(object, file_type)
                    print("", url)
                    if not check_url(url):
                        continue
                
                    filename = "{path}/{object}_{file_type}.tgz".format(path=output_directory,
                                                                    object=object,
                                                                    file_type=file_type)
                    download_file(url, filename)
                
                    if extract:
                        extract_tgz(filename, output_directory)
                
                except Exception as e:
                    print(f"Fehler bei {object} ({file_type}): {e}")
                    continue