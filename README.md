# CanBee-FSW-2022
The source code of Çankaya University CanBee Team's 2022 Flight Software. A _very_ cleaned-up version.

I've wrote the main developer of this in 2020 and 2021. 2022 competition mostly used the same code with some adjustments made by [Burak Geçer](https://github.com/burakgecer) and Berkay Apalı. [Samet](https://github.com/sametefekan) and I wrote the PID code for gimbal, which is probably the most complicated thing we had this year. 

Note that the code is not in a good shape since its development was very time constrained and involved a ton of trial-and-error.

Some libraries used here are not future-proof, for example the XBee thing does not support the latest versions. Using the official Digi library if it works on Teensy or implementing things from scratch by reading the XBee documentation (which we had to do anyway to wrangle the library to get things just right) is a much better idea.

I'm publishing this as an example for the future CanBee team members, use what works for you and remove what doesn't.
