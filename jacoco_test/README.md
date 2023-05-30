# jacoco test

this demonstrates using JaCoCo to measure test coverage.

There are a few build.gradle changes as specified in the Gradle
[manual](https://docs.gradle.org/current/userguide/jacoco_plugin.html)

```
plugins {
    ...
    id "jacoco"
}
...
test {
    ...
    finalizedBy jacocoTestReport
}
jacocoTestReport {
    dependsOn test
}


```

The default report can be found at file:///./build/reports/jacoco/test/html/index.html