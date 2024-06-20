# control

Solutions to the challenges on https://janismac.github.io/ControlChallenges/

The site was recently updated, but I wanted to continue from its old state, so ran the old version of the website locally.

Used the `027b7d091509f2fdbb9a73ad5b34b111128de7ab` hash from https://github.com/janismac/ControlChallenges

Just opening the index.html doesn't work, I was getting CORS errors and not seeing images. I don't remember the reasoning behind it but it was related to local websites accessed with `file://`. Easiest solution is running an http server on the root folder that serves the images. 

```
python -m http.server
```




