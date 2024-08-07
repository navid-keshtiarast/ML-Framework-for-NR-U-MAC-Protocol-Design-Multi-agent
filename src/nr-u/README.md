3GPP NR-U ns-3 module        {#nrureadme}
=====================

This is an [ns-3](https://www.nsnam.org "ns-3 Website") 3GPP NR-U module for the
simulation of NR-U non-standalone cellular networks. Ns-3 is used as a base,
on top of which we will add our module as plug-in (with limitations that will
be discussed below).

## Installation for an authorized developer

As a precondition to the following steps, you must have a working installation 
of ns-3 and nr. If that is the case, then, your local git repo is ready to include
our nr-u module (only for authorized users):

```
$ cd src
$ git clone git@gitlab.com:cttc-lena/nr-u.git
$ cd ..
```

Please note that the src/nr-u directory will be listed as "Untracked files" every
time you do a `git status` command. Ignore it, as the directory lives as an
independent module. As a result, we have now two parallel repositories, but one
lives inside the other. We are working in getting nr working in the contrib/
directory, as per standard ns-3 rules. If you do not have ns-3 or NR installed,
please refer to this [guide](https://cttc-lena.gitlab.io/nr/getting-started.html).

### Test the NR-U installation

Let's configure the project:

```
$ ./waf configure --enable-examples --enable-tests
```

If the NR-U module is recognized correctly, you should see "nr-u" in the list of
built modules. If that is not the case, then most probably the previous
point failed. Otherwise, you could compile it:

```
$ ./waf
```

If that command returns successfully, Welcome to the NR-U world !

## Features

To see the features, please go to the [official webpage](https://cttc-lena.gitlab.io/5g-lena-website/features/).

## Papers

An updated list of published papers that are based on the outcome 
of this module is available [here](https://cttc-lena.gitlab.io/5g-lena-website/papers/).

## Future work

## About

The Mobile Networks group in CTTC is a group of highly skilled researchers, 
with expertise in the area of mobile and computer networks, 
ML/AI based network management, 
SDN/NFV, energy management, performance evaluation. 
Our work on coexistence performance evaluation started with the design and development 
of the LTE-U and LAA extensions of ns-3 LTE module.

We are [on the web](https://cttc-lena.gitlab.io/5g-lena-website/about/).

## Authors ##

In alphabetical order:

- Biljana Bojovic
- Katerina Koutlia
- Lorenza Giupponi
- Sandra Lagen
- Natale Patriciello
- Zoraze Ali

## License ##

This software is licensed under the terms of the GNU GPLv2, as like as ns-3.
See the LICENSE file for more details.
