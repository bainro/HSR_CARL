'''
Coverts a rosbag into a dataset for Perspective Transorming VAE(s)
'''

if __name__ == "__main__":
  # laod map.pgm from sys.argv
  # load rosbag from sys.argv
  # get a path of length X
  # overlay this path on the map 
  # use keys to translate & rotate the path
  # add buffer to map to enable rotate + crop
  # crop with robot position at center
  # rotate, then crop to final size
  # draw robot's pose (start simple, like an arrow)
  # restart and loop over the whole bag
  # save in the format Tim's already using (i.e. csv)
  # save cropped image of map & resized camera image
  
  # === FUTURE FEATURES ===
  # Include history channels (i.e. multiple stacked FPV images)
  # Generating GMP images using the blueprint. 
  # ^ Requires scaling offline or as additional script setting.
