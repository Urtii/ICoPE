import rosbag

with rosbag.Bag('noisy_dataset_header_t.bag', 'w') as outbag:
	for topic, msg, t in rosbag.Bag('noisy_dataset.bag').read_messages():
		if topic == "/tf" and msg.transforms:
			outbag.write(topic, msg, msg.transforms[0].header.stamp)
		else:
			outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
