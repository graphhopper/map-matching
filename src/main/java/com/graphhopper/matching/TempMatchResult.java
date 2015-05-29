package com.graphhopper.matching;

import com.sun.org.apache.xerces.internal.impl.xpath.regex.Match;

import java.util.List;

public class TempMatchResult {
    private MatchResult matchResult;
    private int endIndex;

    public TempMatchResult(MatchResult edgeMatches, int endIndex) {

        this.matchResult = edgeMatches;
        this.endIndex = endIndex;
    }

    public MatchResult getMatchResult() { return matchResult; }
    public int getEndIndex() { return endIndex; }
}
